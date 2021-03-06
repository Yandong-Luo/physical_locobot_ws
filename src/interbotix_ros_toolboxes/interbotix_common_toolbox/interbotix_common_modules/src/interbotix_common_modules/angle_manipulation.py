# Majority of code copied from https://www.learnopencv.com/rotation-matrix-to-euler-angles/

import numpy as np
import math

from geometry_msgs.msg import Quaternion, QuaternionStamped

# Inverts a homogeneous transformation matrix
def transInv(T):
    R, p = T[:3,:3], T[:3, 3]
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]

# Calculates 2D Rotation Matrix given a desired yaw angle
def yawToRotationMatrix(yaw):
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw)],
                    [math.sin(yaw),    math.cos(yaw)],
                    ])
    return R_z

# Transform a Six Element Pose vector to a Transformation Matrix
def poseToTransformationMatrix(pose):
    mat = np.identity(4)
    mat[:3, :3] = eulerAnglesToRotationMatrix(pose[3:])
    mat[:3, 3] = pose[:3]
    return mat

# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta):

    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])

    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, like=R)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    R = np.asarray(R)
    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def quaternion_is_valid(quat, tol=10e-3):
    """Tests if a quaternion is valid
    
    :param quat: Quaternion to check validity of
    :type quat: geometry_msgs.msg.Quaternion
    :param tol: tolerance with which to check validity
    :tpe tol: float
    :return: `True` if quaternion is valid, `False` otherwise
    :rtype: bool
    """
    if isinstance(quat, Quaternion):
        return abs((quat.w * quat.w
            + quat.x * quat.x
            + quat.y * quat.y
            + quat.z * quat.z) - 1.0) < tol
    elif isinstance(quat, QuaternionStamped):
        return abs((quat.quaternion.w * quat.quaternion.w
            + quat.quaternion.x * quat.quaternion.x
            + quat.quaternion.y * quat.quaternion.y
            + quat.quaternion.z * quat.quaternion.z) - 1.0) < tol
    else:
        raise TypeError(
            ("quaternion_is_valid must be given a Quaternion or "
            "QuaternionStamped message. Was given %s." % type(quat)))
