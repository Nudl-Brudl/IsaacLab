import torch
import numpy as np
import isaaclab.utils.math as math_utils

x_0 = torch.tensor([[1, 0, 0]])
y_1 = torch.tensor([[0, 1, 0]])
print(f"Shape of x0 = {x_0.shape}. Shape of y1 = {y_1.shape}")

# compute rotation quaternion of unit
q_0 = math_utils.quat_from_euler_xyz(torch.tensor([[0]]),
                                     torch.tensor([[0]]),
                                     torch.tensor([[0]]))
print(f"Initial orientation = {q_0}")
print(f"Initial orientation shape = {q_0.shape}")




# compute quaternion of rotation 90 in z
q_1 = math_utils.quat_from_euler_xyz(torch.tensor([[0]]),
                                     torch.tensor([[0]]),
                                     torch.tensor([[-np.pi/2]]))
print(f"Second orientation = {q_1}")
print(f"Second orientation shape = {q_1.shape}")
# compute pose_error
e_p, e_angle = math_utils.compute_pose_error(x_0, q_0.view(1, 4), y_1, q_1.view(1, 4))

print(f"Translational error = {e_p}")
print(f"Rotational error = {e_angle}")
