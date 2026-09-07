"""Pose mapping extracted from HC-teleop-robotic/adapters/core/arm_teleop_math.py.

No robot model, joint state, forward kinematics or inverse kinematics is used here.
Quaternions use XYZW; axis_mapping expresses VR increments in the target base.
"""
from __future__ import annotations
import math
from typing import Sequence
import numpy as np


def normalize_quaternion(value: Sequence[float]) -> np.ndarray:
    quaternion = np.asarray(value, dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if quaternion.shape != (4,) or not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    return quaternion / norm


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> np.ndarray:
    x1, y1, z1, w1 = normalize_quaternion(left)
    x2, y2, z2, w2 = normalize_quaternion(right)
    return normalize_quaternion(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def quaternion_inverse(value: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(value)
    return np.asarray([-x, -y, -z, w])


def quaternion_from_axis_angle(axis: Sequence[float], angle: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(vector))
    if vector.shape != (3,) or not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("axis must be a finite non-zero 3D vector")
    vector /= norm
    half = float(angle) * 0.5
    return normalize_quaternion([*(vector * math.sin(half)), math.cos(half)])


def quaternion_to_matrix(value: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(value)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion(matrix: Sequence[Sequence[float]]) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation matrix must be finite 3x3")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        value = [
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ]
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            value = [
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ]
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            value = [
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ]
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            value = [
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ]
    return normalize_quaternion(value)


def orientation_error(target: Sequence[float], current: Sequence[float]) -> np.ndarray:
    difference = quaternion_multiply(target, quaternion_inverse(current))
    if difference[3] < 0.0:
        difference = -difference
    vector_norm = float(np.linalg.norm(difference[:3]))
    if vector_norm < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, float(difference[3]))
    return difference[:3] / vector_norm * angle


def stabilize_pose(
    target_position: Sequence[float],
    target_orientation: Sequence[float],
    previous_position: Sequence[float],
    previous_orientation: Sequence[float],
    position_deadband: float,
    orientation_deadband: float,
    filter_alpha: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply radial deadbands and low-pass smoothing to a pose target."""
    if position_deadband < 0.0 or orientation_deadband < 0.0:
        raise ValueError("pose deadbands must be non-negative")
    if not 0.0 < filter_alpha <= 1.0:
        raise ValueError("filter_alpha must be in (0, 1]")

    target_position_array = np.asarray(target_position, dtype=float)
    previous_position_array = np.asarray(previous_position, dtype=float)
    position_delta = target_position_array - previous_position_array
    position_distance = float(np.linalg.norm(position_delta))
    if position_distance <= position_deadband:
        position = previous_position_array.copy()
    else:
        effective_delta = position_delta * (
            (position_distance - position_deadband) / position_distance
        )
        position = previous_position_array + filter_alpha * effective_delta

    previous_quaternion = normalize_quaternion(previous_orientation)
    rotation_delta = orientation_error(target_orientation, previous_quaternion)
    rotation_angle = float(np.linalg.norm(rotation_delta))
    if rotation_angle <= orientation_deadband:
        orientation = previous_quaternion
    else:
        step_angle = filter_alpha * (rotation_angle - orientation_deadband)
        step = quaternion_from_axis_angle(rotation_delta / rotation_angle, step_angle)
        orientation = quaternion_multiply(step, previous_quaternion)
    return position, orientation


def relative_target(
    vr_position: Sequence[float],
    vr_orientation: Sequence[float],
    reference_vr_position: Sequence[float],
    reference_vr_orientation: Sequence[float],
    reference_robot_position: Sequence[float],
    reference_robot_orientation: Sequence[float],
    axis_mapping: Sequence[Sequence[float]],
    position_scale: float,
    max_displacement: float,
    orientation_enabled: bool,
) -> tuple[np.ndarray, np.ndarray]:
    mapping = np.asarray(axis_mapping, dtype=float)
    if mapping.shape != (3, 3) or not np.all(np.isfinite(mapping)):
        raise ValueError("axis_mapping must be finite 3x3")
    delta = mapping @ (
        np.asarray(vr_position, dtype=float)
        - np.asarray(reference_vr_position, dtype=float)
    ) * float(position_scale)
    length = float(np.linalg.norm(delta))
    if max_displacement > 0.0 and length > max_displacement:
        delta *= max_displacement / length
    target_position = np.asarray(reference_robot_position, dtype=float) + delta

    target_orientation = normalize_quaternion(reference_robot_orientation)
    if orientation_enabled:
        vr_delta = quaternion_multiply(
            vr_orientation, quaternion_inverse(reference_vr_orientation)
        )
        mapped_delta = mapping @ quaternion_to_matrix(vr_delta) @ mapping.T
        target_orientation = quaternion_multiply(
            matrix_to_quaternion(mapped_delta), target_orientation
        )
    return target_position, target_orientation
