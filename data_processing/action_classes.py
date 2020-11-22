import copy
import json
import math
import tf.transformations as trans
import os
import numpy as np

import trimesh
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion


# https://stackoverflow.com/questions/354038/how-do-i-check-if-a-string-is-a-number-float
def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False
    except TypeError:
        return False

class ActionSequence(object):
    def __init__(self, scene, question, actions=[]):
        self.actions = actions  # type: list[Action]
        self.scene = scene
        self.question = question
        self.exec_ind = 0

    def print_seq(self):
        for act in self.actions:
            print(act)

    def __repr__(self):
        s = '' + self.question + '\n'
        for act in self.actions:
            s += act.__repr__() + '\n'
        return s

    @staticmethod
    def load_action_sequence(file=None, question_id=0):
        # Opening JSON file

        base_path = os.path.dirname(os.path.abspath(__file__))
        if file is None:
            path = os.path.join(base_path, 'SHOP_VRB_questions_GT_sequences.json')
        else:
            path = os.path.join(file)
        f = open(path)

        # returns JSON object as
        # a dictionary
        data = json.load(f)
        f.close()

        for x in data['questions']:
            if x['question_index'] == question_id:
                print("i found it!")
                q = x
                break
        else:
            raise NameError('Question with id {} not present.'.format(question_id))
            x = None

        # q = data['questions'][question_id]
        a_seq = ActionSequence(scene=q['image_index'], question=q['question'])
        for act in q['action_sequence']:
            print(act)
            action = None
            split_str = str(act).split('::')
            if split_str.__len__() == 3:
                if split_str[0] == 'move_down':
                    action = Action(actionType='ManipulatorActions.stack({},{})'.format('objects[{}]'.format(int(split_str[1])),
                                                                                        'objects[{}]'.format(int(split_str[2]))))
            if split_str.__len__() == 2:
                if split_str[0] == 'avoid':
                    action = Action(actionType='ManipulatorActions.avoid({})'.format('objects[{}]'.format(int(split_str[1]))))
                if split_str[0] == 'measure' and split_str[1] == 'stiffness':
                    action = Action(actionType='GripperActions.measure_stiffness()')
                if split_str[0] == 'measure' and split_str[1] == 'weight':
                    action = Action(actionType='ManipulatorActions.measure_weight()')
                if split_str[0] == 'move' and split_str[1] == 'up':
                    action = Action(actionType='ManipulatorActions.move_up()')
                if split_str[0] == 'move' and split_str[1] == 'back':
                    action = Action(actionType='ManipulatorActions.move_rel("back")')
                if split_str[0] == 'move' and split_str[1] == 'left':
                    action = Action(actionType='ManipulatorActions.move_rel("left")')
                if split_str[0] == 'move' and split_str[1] == 'forward':
                    action = Action(actionType='ManipulatorActions.move_rel("forward")')
                if split_str[0] == 'move' and split_str[1] == 'front':
                    action = Action(actionType='ManipulatorActions.move_rel("forward")')
                if split_str[0] == 'move' and split_str[1] == 'right':
                    action = Action(actionType='ManipulatorActions.move_rel("right")')
                if split_str[0] == 'move' and is_int(split_str[1]):
                    action = Action(actionType='ManipulatorActions.move_above({})'.format('objects[{}]'.format(int(split_str[1]))))
                if split_str[0] == 'approach_grasp':
                    action = Action(actionType='ManipulatorActions.approach_grasp({})'.format('objects[{}]'.format(int(split_str[1]))))
                if split_str[0] == 'grasp':
                    action = Action(actionType='GripperActions.grasp({})'.format('objects[{}]'.format(int(split_str[1]))))
                    action.action_type += '; ManipulatorActions.attach({})'.format('objects[{}]'.format(int(split_str[1])))
            if split_str.__len__() == 1:
                if split_str[0] == 'release':
                    action = Action(actionType='ManipulatorActions.release()')
                if split_str[0] == 'shake':
                    action = Action(actionType='ManipulatorActions.shake()')
                if split_str[0] == 'flip':
                    action = Action(actionType='ManipulatorActions.flip()')
                if split_str[0] == 'open':
                    action = Action(actionType='ManipulatorActions.open()')
                if split_str[0] == 'close':
                    action = Action(actionType='ManipulatorActions.close()')
                if split_str[0] == 'rotate':
                    action = Action(actionType='ManipulatorActions.rotate()')

            if action is None:
                raise NotImplementedError('action not implemented: {}'.format(act))

            a_seq.actions.append(action)

        print(a_seq)
        return a_seq

# https://stackoverflow.com/questions/354038/how-do-i-check-if-a-string-is-a-number-float
def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False
    except TypeError:
        return False

class Action(object):
    def __init__(self, actionType, par=None):
        self.action_type = 'res, msg = ' + actionType
        if is_int(par):
            self._object_acted_on = int(par)
        else:
            self._kwarg = str(par)

    def __repr__(self):
        return self.action_type


class myObject(object):
    def __init__(self, ID, fname, pos, rot, bb_dim, bb_pos, scale, gripper_setting=0.5, orientation=Quaternion(0,0,0,1)):
        self.ID = ID
        self.fname = fname
        self.pos = pos
        self.rot = rot
        self.bb_dim = bb_dim
        self.bb_pos = bb_pos
        self.bb_rot = [0, 0, 0, 1]
        self.scale = scale
        self.gripper_setting = gripper_setting
        self.orientation = orientation
        # TODO: mesh


    def getPose(self, pos_offset=[0,0,0], euler_offset=[0,0,0]):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position = Point(self.bb_pos[0] + pos_offset[0],
                                   self.bb_pos[1] + pos_offset[1],
                                   self.bb_pos[2] + pos_offset[2])
        quat = trans.quaternion_from_euler(0+euler_offset[0], 0+euler_offset[1], math.pi/180 * self.rot+euler_offset[2])
        pose.pose.orientation = Quaternion(quat[0],
                                           quat[1],
                                           quat[2],
                                           quat[3])
        return pose

    def getGraspPose(self, gripper_depth):
        pose = self.getPose([0, math.pi, -math.pi / 2])
        pose.pose.position.z += self.bb_dim[2] - gripper_depth
        return pose


def getGripperSetting(fname):
    gs = {'knife': 0.95, 'fork': 0.95, 'spoon': 0.95, 'scissors': 0.95}

    try:
        return gs[fname]
    except KeyError:
        return 0.5


class V2aScene(object):
    def __init__(self, scene_id):
        self.objects = []  # type: list[myObject]
        self.scene_id = scene_id

    def print_objects(self):
        for obj in self.objects:
            print(obj.fname, '@ {}'.format(obj.pos))

    def __repr__(self):
        s = ''
        for obj in self.objects:
            s += str(obj.ID) + ': '+ obj.fname+ '@ {}'.format(obj.pos) + '\n'
        return s

    @staticmethod
    def load_scene(path=None, scene_id=0):
        scene = V2aScene(scene_id=scene_id)
        base_path = os.path.dirname(os.path.abspath(__file__))
        if path is None:
            path = os.path.join(base_path, 'SHOP_VRB_scenes_V2A.json')
        f = open(path)
        data = json.load(f)


        scene.objects = []
        for idx, obj in enumerate(data['scenes'][scene_id]['objects']):
            index = idx
            fname = str(obj['file'])
            point = obj['3d_coords']
            rotation = obj['rotation']
            rot = obj['orientation']
            quat = Quaternion(rot[1], rot[2], rot[3], rot[0])
            #
            # bb_dim = [obj['bbox']['x'], obj['bbox']['y'], obj['bbox']['z']]
            # bb_pos = copy.copy(point)
            # bb_pos[2] += 0.5*bb_dim[2]  #TODO: thig objects is only valid for standing objects
            vert = np.linspace(0, 7, 7, dtype=int)
            mesh = trimesh.Trimesh(vertices=obj['bbox_robot_coords'],
                                   faces=[[t1, t2, t3] for t1, t2, t3 in zip(vert[0:-2], vert[1:-1], vert[2:])])
            bb_dim = mesh.bounding_box.extents
            bb_pos = mesh.bounding_box.center_mass
            # bb_rot = [0, 0, 0, 1.0]
            scale = obj['scale_factor']

            scene.objects.append(myObject(index, fname, point, rotation, bb_dim, bb_pos, scale, orientation=quat, gripper_setting=getGripperSetting(fname)))


        for obj in scene.objects:
            print(obj.ID, ': {}'.format(obj.fname))

        return scene