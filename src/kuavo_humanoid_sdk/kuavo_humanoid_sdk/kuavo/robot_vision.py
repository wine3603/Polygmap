#!/usr/bin/env python3
# coding: utf-8
import math
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.interfaces.data_types import (AprilTagData)
from kuavo_humanoid_sdk.kuavo.core.ros.vision import KuavoRobotVisionCore

try:
    from apriltag_ros.msg import AprilTagDetectionArray
except ImportError:
    pass

class KuavoRobotVision:
    """Vision system interface for Kuavo humanoid robot.
    
    Provides access to AprilTag detection data from different coordinate frames.
    
    """
    
    def __init__(self, robot_type: str = "kuavo"):
        """Initialize the vision system.
        
        Args:
            robot_type (str, optional): Robot type identifier. Defaults to "kuavo".
        """
        if not hasattr(self, '_initialized'):
            self._vision_core = KuavoRobotVisionCore()

    def get_data_by_id(self, target_id: int, data_source: str = "base") -> dict:
        """Get AprilTag detection data for a specific ID from specified source.
        
        Args:
            target_id (int): AprilTag ID to retrieve
            data_source (str, optional): Data source frame. Can be "base", "camera", 
                or "odom". Defaults to "base".
                
        Returns:
            dict: Detection data containing position, orientation and metadata
        """
        return self._vision_core._get_data_by_id(target_id, data_source)
    
    def get_data_by_id_from_camera(self, target_id: int) -> dict:
        """Get AprilTag data from camera coordinate frame.
        
        Args:
            target_id (int): AprilTag ID to retrieve
            
        Returns:
            dict: See get_data_by_id() for return format
        """
        return self._vision_core._get_data_by_id(target_id, "camera")
    
    def get_data_by_id_from_base(self, target_id: int) -> dict:
        """Get AprilTag data from base coordinate frame.
        
        Args:
            target_id (int): AprilTag ID to retrieve
            
        Returns:
            dict: See get_data_by_id() for return format
        """
        return self._vision_core._get_data_by_id(target_id, "base")
    
    def get_data_by_id_from_odom(self, target_id: int) -> dict:
        """Get AprilTag data from odom coordinate frame.
        
        Args:
            target_id (int): AprilTag ID to retrieve
            
        Returns:
            dict: See get_data_by_id() for return format
        """
        return self._vision_core._get_data_by_id(target_id, "odom")
    
    @property
    def apriltag_data_from_camera(self) -> AprilTagData:
        """AprilTagData: All detected AprilTags in camera frame (property)"""
        return self._vision_core.apriltag_data_from_camera
    
    @property
    def apriltag_data_from_base(self) -> AprilTagData:
        """AprilTagData: All detected AprilTags in camera frame (property)"""
        return self._vision_core.apriltag_data_from_base
    
    @property
    def apriltag_data_from_odom(self) -> AprilTagData:
        """AprilTagData: All detected AprilTags in camera frame (property)"""
        return self._vision_core.apriltag_data_from_odom
    

