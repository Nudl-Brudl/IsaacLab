from __future__ import annotations

import torch
from tensordict import TensorDict
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from omni.isaac.lab.envs import DirectRLEnv


# =========================================================
# PRECOMPUTED CONSTANTS (put this once, not inside function)
# =========================================================

# ---- left-right permutation (actions and joints) ----
LR_PERM = torch.tensor([
    0, 1,        # waist
    4, 5,        # right_upper_arm -> left
    2, 3,        # left_upper_arm -> right
    6,           # pelvis
    8, 7,        # lower arms
    12, 13, 14,  # right thigh -> left
    9, 10, 11,   # left thigh -> right
    16, 15,      # knees
    19, 20,      # right foot -> left
    17, 18       # left foot -> right
], dtype=torch.long)


# ---- sign flips for left-right reflection ----
LR_SIGN = torch.ones(21)

# hips (x and z flip)
LR_SIGN[[9, 11, 12, 14]] = -1

# upper arms (first DOF typically lateral)
LR_SIGN[[2, 4]] = -1

# feet (first DOF typically roll-like)
LR_SIGN[[17, 19]] = -1

# convert once to avoid recreating
LR_SIGN = LR_SIGN


# =========================================================
# MAIN FUNCTION
# =========================================================

@torch.no_grad()
def compute_symmetric_obs_actions(
    env: DirectRLEnv,
    obs: TensorDict | None = None, 
    actions: torch.Tensor | None = None
):
    
    if obs is not None:
        B = obs.batch_size[0]
        obs_aug = obs.repeat(3)
        device = obs["policy"].device
        # --original
        obs_aug["policy"][:B] = obs["policy"]

        # --left-right 
        obs_lr = obs["policy"].clone()

        # --- body linear velocity (x,y,z) ---
        obs_lr[:, 1:4] *= torch.tensor([1, -1, 1], device=device)

        # --- angular velocity ---
        obs_lr[:, 4:7] *= torch.tensor([-1, 1, -1], device=device)

        # --- orientation ---
        obs_lr[:, 7] *= -1  # yaw
        obs_lr[:, 8] *= -1  # roll
        obs_lr[:, 9] *= -1  # angle to target

        # --- joints (positions + velocities) ---
        j0 = 12
        n = 21

        # positions
        joint_pos = obs_lr[:, j0:j0+n]
        joint_pos = joint_pos[:, LR_PERM] * LR_SIGN.to(device)

        # velocities
        joint_vel = obs_lr[:, j0+n:j0+2*n]
        joint_vel = joint_vel[:, LR_PERM] * LR_SIGN.to(device)

        obs_lr[:, j0:j0+n] = joint_pos
        obs_lr[:, j0+n:j0+2*n] = joint_vel

        obs_aug["policy"][B:2*B] = obs_lr

        obs_yaw = obs["policy"].clone()

        # -- yaw 180
        # --- velocities ---
        obs_yaw[:, 1:4] *= -1
        obs_yaw[:, 4:7] *= -1

        # --- yaw ---
        obs_yaw[:, 7] = torch.atan2(
            torch.sin(obs_yaw[:, 7] + torch.pi),
            torch.cos(obs_yaw[:, 7] + torch.pi),
        )

        # --- angle to target ---
        obs_yaw[:, 9] = torch.atan2(
            torch.sin(obs_yaw[:, 9] + torch.pi),
            torch.cos(obs_yaw[:, 9] + torch.pi),
        )

        # joints unchanged
        obs_aug["policy"][2*B:] = obs_yaw
    else:
        obs_aug = None


    if actions is not None:
            
        B = actions.shape[0]
        device = actions.device

        # expand
        
        act_aug = torch.empty(3 * B, actions.shape[1], device=device)

        # -----------------------------------------------------
        # ORIGINAL
        # -----------------------------------------------------
        act_aug[:B] = actions

        # -----------------------------------------------------
        # LEFT-RIGHT
        # --- actions ---
        act_lr = actions[:, LR_PERM] * LR_SIGN.to(device)
        act_aug[B:2*B] = act_lr

        # -----------------------------------------------------
        # YAW 180°
        # -----------------------------------------------------
        # actions unchanged
        act_aug[2*B:] = actions
    else:
        act_aug = None

    return obs_aug, act_aug