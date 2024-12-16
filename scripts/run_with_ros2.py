import argparse
import time

from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Load Robot With Robot Control")
parser.add_argument(
    "--robot",
    type=str,
    choices=["anymal_d", "h1", "g1"],
    default="h1",
    help="Choose which robot to load: anymal_d, h1, or g1.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass
from tju.benchmark.ros2control import SimControl
##
# Pre-defined configs
##
from omni.isaac.lab_assets import G1_MINIMAL_CFG, H1_MINIMAL_CFG  # isort:skip


@configclass
class RobotSceneCfg(InteractiveSceneCfg):
    """Configuration for a simple scene with a robot."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # articulation
    if args_cli.robot == "h1":
        robot: ArticulationCfg = H1_MINIMAL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    elif args_cli.robot == "g1":
        robot: ArticulationCfg = G1_MINIMAL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    else:
        raise ValueError(f"Unsupported robot type: {args_cli.robot}.")


def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
    scene_cfg = RobotSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    control = SimControl(sim, scene, args_cli.robot)
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()