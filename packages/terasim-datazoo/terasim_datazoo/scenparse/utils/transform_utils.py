# Licensed under the CC BY-NC 4.0 license (https://creativecommons.org/licenses/by-nc/4.0/)
import numpy as np
import torch
from torch import Tensor
import transforms3d
from typing import Union


def cast_rad(angle: Union[float, np.ndarray, Tensor]) -> Union[float, np.ndarray, Tensor]:
    """Cast angle such that they are always in the [-pi, pi) range."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _rotation33_as_yaw(rotation: np.ndarray) -> float:
    """Compute the yaw component of given 3x3 rotation matrix.

    Args:
        rotation (np.ndarray): 3x3 rotation matrix (np.float64 dtype recommended)

    Returns:
        float: yaw rotation in radians
    """
    return transforms3d.euler.mat2euler(rotation)[2]


def _yaw_as_rotation33(yaw: float) -> np.ndarray:
    """Create a 3x3 rotation matrix from given yaw.
    The rotation is counter-clockwise and it is equivalent to:
    [cos(yaw), -sin(yaw), 0.0],
    [sin(yaw), cos(yaw), 0.0],
    [0.0, 0.0, 1.0],

    Args:
        yaw (float): yaw rotation in radians

    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    return transforms3d.euler.euler2mat(0, 0, yaw)


def get_so2_from_se2(transform_se3: np.ndarray) -> np.ndarray:
    """Gets rotation component in SO(2) from transformation in SE(2).

    Args:
        transform_se3: se2 transformation.

    Returns:
        rotation component in so2
    """
    rotation = np.eye(3, dtype=np.float64)
    rotation[:2, :2] = transform_se3[:2, :2]
    return rotation


def get_yaw_from_se2(transform_se3: np.ndarray) -> float:
    """Gets yaw from transformation in SE(2).

    Args:
        transform_se3: se2 transformation.

    Returns:
        yaw
    """
    return _rotation33_as_yaw(get_so2_from_se2(transform_se3))


def transform_points(points: np.ndarray, transf_matrix: np.ndarray) -> np.ndarray:
    """Transform points using transformation matrix.
    Note this function assumes points.shape[1] == matrix.shape[1] - 1, which means that the last
    row in the matrix does not influence the final result.
    For 2D points only the first 2x3 part of the matrix will be used.

    Args:
        points (np.ndarray): Input points (Nx2) or (Nx3).
        transf_matrix (np.ndarray): np.float64, 3x3 or 4x4 transformation matrix for 2D and 3D input respectively

    Returns:
        np.ndarray: array of shape (N,2) for 2D input points, or (N,3) points for 3D input points
    """
    assert len(points.shape) == len(transf_matrix.shape) == 2, (
        f"dimensions mismatch, both points ({points.shape}) and "
        f"transf_matrix ({transf_matrix.shape}) needs to be 2D numpy ndarrays."
    )
    assert (
        transf_matrix.shape[0] == transf_matrix.shape[1]
    ), f"transf_matrix ({transf_matrix.shape}) should be a square matrix."

    if points.shape[1] not in [2, 3]:
        raise AssertionError(f"Points input should be (N, 2) or (N, 3) shape, received {points.shape}")

    assert points.shape[1] == transf_matrix.shape[1] - 1, "points dim should be one less than matrix dim"

    points_transformed = (points @ transf_matrix.T[:-1, :-1]) + transf_matrix[:-1, -1]

    return points_transformed.astype(points.dtype)


def get_transformation_matrix(agent_translation_m: np.ndarray, agent_yaw: float) -> np.ndarray:
    """Get transformation matrix from world to vehicle frame

    Args:
        agent_translation_m (np.ndarray): (x, y) position of the vehicle in world frame
        agent_yaw (float): rotation of the vehicle in the world frame

    Returns:
        (np.ndarray) transformation matrix from world to vehicle
    """

    # Translate world to ego by applying the negative ego translation.
    world_to_agent_in_2d = np.eye(3, dtype=np.float64)
    world_to_agent_in_2d[0:2, 2] = -agent_translation_m[0:2]

    # Rotate counter-clockwise by negative yaw to align world such that ego faces right.
    world_to_agent_in_2d = _yaw_as_rotation33(-agent_yaw) @ world_to_agent_in_2d

    return world_to_agent_in_2d


# transformation for torch
def torch_rad2rot(rad: Tensor) -> Tensor:
    """
    Args:
        rad: [n_batch] or [n_scene, n_agent] or etc.

    Returns:
        rot_mat: [{rad.shape}, 2, 2]
    """
    _cos = torch.cos(rad)
    _sin = torch.sin(rad)
    return torch.stack([torch.stack([_cos, -_sin], dim=-1), torch.stack([_sin, _cos], dim=-1)], dim=-2)


def torch_sincos2rot(in_sin: Tensor, in_cos: Tensor) -> Tensor:
    """
    Args:
        in_sin: [n_batch] or [n_scene, n_agent] or etc.
        in_cos: [n_batch] or [n_scene, n_agent] or etc.

    Returns:
        rot_mat: [{in_sin.shape}, 2, 2]
    """
    return torch.stack([torch.stack([in_cos, -in_sin], dim=-1), torch.stack([in_sin, in_cos], dim=-1)], dim=-2)


def torch_pos2local(in_pos: Tensor, local_pos: Tensor, local_rot: Tensor) -> Tensor:
    """Transform M position to the local coordinates.

    Args:
        in_pos: [..., M, 2]
        local_pos: [..., 1, 2]
        local_rot: [..., 2, 2]

    Returns:
        out_pos: [..., M, 2]
    """
    return torch.matmul(in_pos - local_pos, local_rot)


def torch_pos2global(in_pos: Tensor, local_pos: Tensor, local_rot: Tensor) -> Tensor:
    """Reverse torch_pos2local

    Args:
        in_pos: [..., M, 2]
        local_pos: [..., 1, 2]
        local_rot: [..., 2, 2]

    Returns:
        out_pos: [..., M, 2]
    """
    return torch.matmul(in_pos, local_rot.transpose(-1, -2)) + local_pos


def torch_dir2local(in_dir: Tensor, local_rot: Tensor) -> Tensor:
    """Transform M dir to the local coordinates.

    Args:
        in_dir: [..., M, 2]
        local_rot: [..., 2, 2]

    Returns:
        out_dir: [..., M, 2]
    """
    return torch.matmul(in_dir, local_rot)


def torch_dir2global(in_dir: Tensor, local_rot: Tensor) -> Tensor:
    """Reverse torch_dir2local

    Args:
        in_dir: [..., M, 2]
        local_rot: [..., 2, 2]

    Returns:
        out_dir: [..., M, 2]
    """
    return torch.matmul(in_dir, local_rot.transpose(-1, -2))


def torch_rad2local(in_rad: Tensor, local_rad: Tensor, cast: bool = True) -> Tensor:
    """Transform M rad angles to the local coordinates.

    Args:
        in_rad: [..., M]
        local_rad: [...]

    Returns:
        out_rad: [..., M]
    """
    out_rad = in_rad - local_rad.unsqueeze(-1)
    if cast:
        out_rad = cast_rad(out_rad)
    return out_rad


def torch_rad2global(in_rad: Tensor, local_rad: Tensor) -> Tensor:
    """Reverse torch_rad2local

    Args:
        in_rad: [..., M]
        local_rad: [...]

    Returns:
        out_rad: [..., M]
    """
    return cast_rad(in_rad + local_rad.unsqueeze(-1))
