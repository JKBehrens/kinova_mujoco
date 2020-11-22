#!/usr/bin/env python
"""
Copyright (c) 2020 Jan Behrens
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""
import os
from collections import namedtuple
from rospkg import RosPack

"""
Export the state of the world using forward kinematics and object pose service
"""

import rospy
from kinova_mujoco.kinematics_interface import ForwardKinematics, MoveItErrorCodes
# from mujoco_interface_msgs.srv import GetAllObjectPosesResponse, GetAllObjectPosesRequest, GetAllObjectPoses
from mujoco_interface_msgs.srv import GetObjectPoseRequest, GetObjectPose, GetObjectPoseResponse
from urdf_parser_py.urdf import URDF
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
import json
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from rospy_message_converter import message_converter

URDF_PGK = "kinova_mujoco"
URDF_FILE = "world_model/kinova_fel_with_objects_NoDae.urdf"
robot = None

mg = MoveGroupCommander('arm')
mg_gripper = MoveGroupCommander('gripper')
mg.set_pose_reference_frame('base_link')
rc = RobotCommander()

arm_links = rc.get_link_names('arm')
gripper_links = rc.get_link_names('gripper')


VisElement = namedtuple('VisElement', ['name', 'geometry', 'pose'])

if __name__ == "__main__":
    rospy.init_node('scene_exporter')

    srv = rospy.ServiceProxy('/kinova_mujoco/getObjectPose',
                             GetObjectPose)
    fk = ForwardKinematics()

    rospack = RosPack()
    # rospack.get_path(URDF_PGK)
    SCENE_PATH = os.path.join(rospack.get_path(URDF_PGK), URDF_FILE)
    robot = URDF.from_xml_file(SCENE_PATH)

    links = []
    for link in robot.links:
        if link.visual is not None:
            links.append(link)
    visElements = []
    scene_dict = {}
    robot_dict = {}
    for link in links:
        if link.name in arm_links + gripper_links + ['base_link']:
            res = fk.getCurrentFK([link.name], 'base_link')  # type: GetPositionFKResponse
            if res.error_code.val == MoveItErrorCodes.SUCCESS:

                # e = VisElement(link.name, link.visual.geometry.filename, res.pose_stamped[0])
                robot_dict[link.name] = message_converter.convert_ros_message_to_dictionary(res)
                # visElements.append(e)

        if 'visual' in link.name:
            res = srv.call(GetObjectPoseRequest(link.name))  # type: GetObjectPoseResponse
            # if res.error_code.val == MoveItErrorCodes.SUCCESS:
            if res.success:
                e = VisElement(link.name, link.visual.geometry.filename, res.pose)
                visElements.append(e)
                scene_dict[link.name] = message_converter.convert_ros_message_to_dictionary(res)



    # print(json.dumps(visElements, indent=4))
    # print(visElements)

    f = open('robot_state.json', 'w+')
    json.dump(robot_dict, f, indent=4)
    f.close()

    f = open('robot.urdf', 'w+')
    f.write(rospy.get_param('robot_description'))
    f.close()

    f = open('scene_state.json', 'w+')
    json.dump(scene_dict, f, indent=4)
    f.close()
    #
    # print(json.dumps(robot_dict, indent=4))
    # print(rospy.get_param('robot_description'))
    # print(json.dumps(scene_dict, indent=4))