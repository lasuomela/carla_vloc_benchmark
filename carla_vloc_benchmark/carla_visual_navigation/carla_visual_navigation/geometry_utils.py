import geometry_msgs
import numpy as np
import transforms3d as t3d
from copy import deepcopy

from rosidl_runtime_py.convert import message_to_ordereddict

def point_msg2np(msg_point):
    np_point = np.array([msg_point.x, msg_point.y, msg_point.z])
    return np_point

def np2point_msg(np_point):
    msg_point = geometry_msgs.msg.Point(x=np_point[0], 
                                        y=np_point[1], 
                                        z=np_point[2])
    return msg_point

def quat_msg2np(msg_quat):
    np_quat = np.array([msg_quat.w,
                        msg_quat.x,
                        msg_quat.y,
                        msg_quat.z ])
    return np_quat

def np2quat_msg(np_quat):
    msg_quat = geometry_msgs.msg.Quaternion(w=np_quat[0],
                                            x=np_quat[1],
                                            y=np_quat[2],
                                            z=np_quat[3])
    return msg_quat

def distance_from_odometry(msg1, msg2):
    pos1 = point_msg2np( msg1.pose.pose.position )
    pos2 = point_msg2np( msg2.pose.pose.position )
    dist = np.linalg.norm(pos1-pos2)
    return dist

def distance_from_pose(msg1, msg2):
    pos1 = point_msg2np( msg1.position )
    pos2 = point_msg2np( msg2.position )
    dist = np.linalg.norm(pos1-pos2)
    return dist

def angular_distance_from_pose(msg1, msg2):
    q_1 = np.array( [ message_to_ordereddict(msg1.orientation)[component] for component in ['w', 'x', 'y', 'z']])
    q_2 = np.array( [ message_to_ordereddict(msg2.orientation)[component] for component in ['w', 'x', 'y', 'z']])
    qd = t3d.quaternions.qmult(q_2, t3d.quaternions.qinverse(q_1))
    qd = qd/t3d.quaternions.qnorm(qd)
    angle, dist = t3d.quaternions.quat2axangle(qd)

    # return the angular distance to the direction with smaller angle
    dist = min(dist/(2*np.pi), 1-dist/(2*np.pi))*360
    return angle, dist