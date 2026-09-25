"""Configuration for ABLE exoskeleton

"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg


ABLE_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path="/workspace/isaaclab/bind_in/able_ros2_description/urdf/able_NO_collision.urdf",
        usd_dir="/workspace/isaaclab/bind_in/able_ros2_description/usd",
        fix_base=False,
        root_link_name="trunk",
        collision_from_visuals=True,
        collider_type="convex_hull",
        self_collision=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=10000, damping=0.2),
            drive_type="force",
        )
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "left_hip": 0.0,
            "left_knee": 0.017,
            "right_hip": 0.0,
            "right_knee": 0.017,
        },
        pos=(0.0, 0.0, 1.5),
        # joint_vel={".*": 0.0},
    ),
    actuators={
        "left_hip_act": ImplicitActuatorCfg(
            joint_names_expr=["left_hip"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "left_knee_act": ImplicitActuatorCfg(
            joint_names_expr=["left_knee"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "right_hip_act": ImplicitActuatorCfg(
            joint_names_expr=["right_hip"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "right_knee_act": ImplicitActuatorCfg(
            joint_names_expr=["right_knee"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
    },
)