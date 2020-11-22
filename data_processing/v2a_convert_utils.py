import json
import math
import os
from mesh_store.voxelgrid2urdf import mesh2urdf


def create_object_scene(objects, scene_urdf=None, scene=0):
    '''
    Make a URDF for the scene composed of objects. The object meshes are decomposed into convex bodies.
    :param objects: list of myObject objects
    :param scene_urdf: ODIO urdf object which should be extended with the objects. If None, we create a
    new URDF (normal use-case)
    :param scene: id of scene (int), which is just used for naming the URDF
    :return: odio urdf description of scene.
    '''
    scene_urdf = None
    for obj in objects:
        scene_urdf = mesh2urdf('package://mesh_store/meshes/' + obj.fname + '.stl',
                  obj_name=obj.fname + '_{}'.format(obj.ID),
                  scale=3 * [obj.scale] + [1.0],
                  maxhulls=50,
                  init_orig=obj.pos + 2* [0] + [obj.rot * math.pi / 180.0],
                  robot_urdf=scene_urdf)


    if os.path.isfile('scene_' + str(scene) + ".urdf"):
        raise IOError('Scene file already exists')
    text_file = open('scene_' + str(scene) + ".urdf", "w")
    n = text_file.write(scene_urdf.__str__())
    text_file.close()
    return scene_urdf