import numpy as np
import base64
import json
from scipy.spatial.transform import Rotation as R

def axis_angle_to_rotation_matrix(x, y, z, angle_degrees):
    # Create a Rotation object from the axis-angle representation
    r = R.from_rotvec(np.deg2rad(angle_degrees) * np.array([x, y, z]))
    # Convert the Rotation object to a rotation matrix
    rot_matrix = r.as_matrix()
    return rot_matrix


def transforms_to_matrix(transforms):
    """
    {
        "axisAngle": {
            "x": -0.6438568693726755,
            "y": -0.6321450501312722,
            "z": 0.4310927595728662,
            "angleDegrees": 133.00126907434236
        },
        "translation": {
            "x": -1.0385507345199585,
            "y": 0.017907384783029556,
            "z": 0.9713748693466187
        }
    }
    """
    # Extract the translation and rotation from the sensorToVehicleTransform
    translation = transforms['translation']
    axis_angle = transforms['axisAngle']

    # Create a rotation matrix from the axis-angle representation
    rot_matrix = axis_angle_to_rotation_matrix(axis_angle.get('x', 0), axis_angle.get('y', 0), axis_angle.get('z', 0), axis_angle.get('angleDegrees', 0))
    # Create a transformation matrix from the rotation matrix and translation vector
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rot_matrix
    transformation_matrix[:3, 3] = np.array([translation.get('x', 0), translation.get('y', 0), translation.get('z', 0)])

    return transformation_matrix


def sensorToVehicleTransform_to_transformation_matrix(sensorToVehicleTransform):
    """
    {
        "axisAngle": {
            "x": -0.6438568693726755,
            "y": -0.6321450501312722,
            "z": 0.4310927595728662,
            "angleDegrees": 133.00126907434236
        },
        "translation": {
            "x": -1.0385507345199585,
            "y": 0.017907384783029556,
            "z": 0.9713748693466187
        }
    }
    """
    return transforms_to_matrix(sensorToVehicleTransform)
