import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a pick and place robot to the scene."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
import isaaclab.sim.utils.prims as prim_utils
from isaaclab.assets import AssetBaseCfg, Articulation
from isaaclab.sim import SimulationContext
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg

from isaaclab_assets import PICK_AND_PLACE_CFG


class MyRobotSceneCfg(InteractiveSceneCfg):
    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", 
                          spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(prim_path="/World/Light",
                              spawn=sim_utils.DomeLightCfg(
                                  intensity=3000.0, color=(0.75, 0.75, 0.75)))

    # robot
    # pap_robot_cfg = PICK_AND_PLACE_CFG.copy()
    # pap_robot_cfg.prim_path = "/World/Origin.*/Robot"
    # pap_robot = Articulation(cfg=pap_robot_cfg)
    pap_robot = PICK_AND_PLACE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    while simulation_app.is_running():
        if count % 500 == 0:
            count = 0

            # put back to initial place
            root_state = scene["pap_robot"].data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins
            scene["pap_robot"].write_root_pose_to_sim(root_state[:, :7])
            scene["pap_robot"].write_root_velocity_to_sim(root_state[:, 7:])

            joint_pos, joint_vel = (scene["pap_robot"].data.default_joint_pos.clone(), 
                                    scene["pap_robot"].data.default_joint_vel.clone())
            joint_pos += torch.rand_like(joint_pos) * 0.1
            scene["pap_robot"].write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            scene["pap_robot"].reset()
            print("[INFO]: Resetting robot state...")

        sim.step()
        count += 1

def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])

    # design scene
    scene_cfg = MyRobotSceneCfg(args_cli.num_envs, env_spacing=8.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()