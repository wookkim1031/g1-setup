#!/usr/bin/env python3
"""Load a scene from build_isaac_scene.py / randomize_layout.py in Isaac Lab, test its colliders with dropped balls and render two views.

    ./isaaclab.sh -p check_isaac_scene.py <hall_scene.usda> <scene_report.json> <out_dir> --headless [--enable_cameras]
    ./isaaclab.sh -p check_isaac_scene.py layouts/layout_0.usda layouts/layout_0.json <out_dir> --headless

A 10 cm ball is dropped over each test point of the report (free floor, a fixed and a movable object box, a static column block);
it has to come to rest on top of that surface. Writes <out>/isaac_check.json and, with --enable_cameras, view_overview.png
and view_robot_start.png (on our server, 2026-11-05, the RTX renderer of Isaac Sim 5.1 segfaults at start-up in the
isaaclab container with --enable_cameras, before this script runs; without it the physics check works).
"""
import argparse, json, os
from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("scene"); ap.add_argument("report"); ap.add_argument("out")
AppLauncher.add_app_launcher_args(ap)
a = ap.parse_args()
simulation_app = AppLauncher(a).app

import numpy as np
import torch
from PIL import Image
from pxr import Usd, UsdGeom, UsdPhysics
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.sensors import Camera, CameraCfg

os.makedirs(a.out, exist_ok=True)
rep = json.load(open(a.report))
R = 0.1
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device=a.device))
cfg = sim_utils.UsdFileCfg(usd_path=os.path.abspath(a.scene)); cfg.func("/World/Hall", cfg)
cfg = sim_utils.DomeLightCfg(intensity=1500.0, color=(1.0, 1.0, 1.0)); cfg.func("/World/Light", cfg)
balls = []
for k, tp in enumerate(rep["test_points"]):
    balls.append(RigidObject(RigidObjectCfg(
        prim_path="/World/Ball%d" % k,
        spawn=sim_utils.SphereCfg(radius=R, rigid_props=sim_utils.RigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                                  collision_props=sim_utils.CollisionPropertiesCfg(), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.1))),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(tp["xy"][0], tp["xy"][1], tp["surface_z"] + 0.6)))))
camera = None if not a.enable_cameras else Camera(CameraCfg(prim_path="/World/CheckCam", update_period=0, height=720, width=1280, data_types=["rgb"],
                          spawn=sim_utils.PinholeCameraCfg(focal_length=14.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 200.0))))
sim.reset()

stage = sim.stage
prims = [p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Hall"))]
objs = [p for p in prims if p.GetParent().GetName() in ("Fixed", "Movable")]
out = {"scene": a.scene, "prims_under_hall": len(prims), "object_prims": len(objs),
       "fixed_prims": sum(p.GetParent().GetName() == "Fixed" for p in objs), "movable_prims": sum(p.GetParent().GetName() == "Movable" for p in objs),
       "object_prims_with_collision": sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in objs),
       "object_prims_with_class_label": sum(bool(p.GetAttribute("semantics:labels:class").Get()) for p in objs),
       "static_mesh_points": len(UsdGeom.Mesh(stage.GetPrimAtPath("/World/Hall/Static")).GetPointsAttr().Get()),
       "static_has_collision": stage.GetPrimAtPath("/World/Hall/Static").HasAPI(UsdPhysics.CollisionAPI),
       "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage), "up_axis": str(UsdGeom.GetStageUpAxis(stage)), "drops": []}

dt = sim.get_physics_dt()
for _ in range(480):  # 4 s
    sim.step()
    for b in balls:
        b.update(dt)
for tp, b in zip(rep["test_points"], balls):
    p = b.data.root_pos_w[0].cpu().numpy()
    d = {"what": tp["what"], "xy": tp["xy"], "surface_z": tp["surface_z"], "ball_rest_z": float(p[2]), "expected_z": tp["surface_z"] + R,
         "moved_xy_m": float(np.hypot(p[0] - tp["xy"][0], p[1] - tp["xy"][1]))}
    d["ok"] = bool(abs(d["ball_rest_z"] - d["expected_z"]) < 0.03)
    out["drops"].append(d)
    print("drop on %-28s rest z %.3f (expected %.3f), slid %.2f m -> %s" % (tp["what"], d["ball_rest_z"], d["expected_z"], d["moved_xy_m"], "ok" if d["ok"] else "FAIL"), flush=True)

lo, hi = np.array(rep["hall_box_scene_m"]["min"]), np.array(rep["hall_box_scene_m"]["max"])
cen = (lo + hi) / 2
views = {"view_overview": ((lo[0] - 4.0, lo[1] - 6.0, 16.0), (cen[0], cen[1], 0.0)),
         "view_robot_start": ((0.0, 0.0, 1.3), tuple(rep["look_from_start"]) + (0.6,))}
for name, (eye, tgt) in (views.items() if camera is not None else ()):
    camera.set_world_poses_from_view(torch.tensor([eye], dtype=torch.float32, device=sim.device), torch.tensor([tgt], dtype=torch.float32, device=sim.device))
    for _ in range(30):  # let the renderer settle
        sim.step()
    camera.update(dt)
    rgb = camera.data.output["rgb"][0].cpu().numpy()[..., :3]
    Image.fromarray(rgb.astype(np.uint8)).save(os.path.join(a.out, name + ".png"))
    out[name] = {"eye": eye, "target": tgt, "mean_brightness": float(rgb.mean()), "std": float(rgb.std())}

json.dump(out, open(os.path.join(a.out, "isaac_check.json"), "w"), indent=1)
print("CHECK DONE", json.dumps({k: v for k, v in out.items() if k != "drops"}), flush=True)
os._exit(0)  # simulation_app.close() spins forever in this container (seen 2026-11-05, 10 min at 100 % CPU)
