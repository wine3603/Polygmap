#!/usr/bin/env python3
# coding: utf-8
import math
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.interfaces.data_types import (PoseQuaternion, HomogeneousMatrix)
from kuavo_humanoid_sdk.kuavo.core.ros.tools import KuavoRobotToolsCore
from typing import Union

class KuavoRobotTools:
    """Robot tools class providing coordinate frame transformation interfaces.
    
    This class encapsulates coordinate transformation queries between different robot frames,
    supporting multiple return data formats.
    """
    
    def __init__(self):
        """Initialize robot tools instance."""
        self.tools_core = KuavoRobotToolsCore()

    def get_tf_transform(self, target_frame: str, source_frame: str, 
                       return_type: str = "pose_quaternion") -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """Get transformation between specified coordinate frames.
        
        Args:
            target_frame (str): Name of target coordinate frame
            source_frame (str): Name of source coordinate frame
            return_type (str, optional): Return data format type. Valid values: 
                "pose_quaternion" - quaternion pose format,
                "homogeneous" - homogeneous matrix format. Defaults to "pose_quaternion".
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: 
                Transformation data in specified format, or None if failed
        
        Raises:
            ValueError: If invalid return_type is provided
        """
        return self.tools_core._get_tf_tree_transform(target_frame, source_frame, return_type=return_type)

    def get_base_to_odom(self, return_type: str = "pose_quaternion") -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """Get transformation from base_link to odom frame.
        
        Args:
            return_type (str, optional): Return format type. Same as get_tf_transform. 
                Defaults to "pose_quaternion".
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: Transformation data or None
        """
        return self.get_tf_transform("odom", "base_link", return_type)

    def get_camera_to_base(self, return_type: str = "homogeneous") -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """Get transformation from camera_link to base_link frame.
        
        Args:
            return_type (str, optional): Return format type. Same as get_tf_transform.
                Defaults to "homogeneous".
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: Transformation data or None
        """
        return self.get_tf_transform("base_link", "camera_link", return_type)
