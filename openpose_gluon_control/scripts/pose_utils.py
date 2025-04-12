#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from geometry_msgs.msg import Pose

def pose_to_numpy(pose):
    """将Pose消息转换为numpy数组"""
    return np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])

def check_workspace_bounds(position, bounds):
    """检查工作空间范围"""
    return all([
        bounds['x'][0] < position.x < bounds['x'][1],
        bounds['y'][0] < position.y < bounds['y'][1],
        bounds['z'][0] < position.z < bounds['z'][1]
    ])
