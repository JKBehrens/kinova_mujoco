#!/usr/bin/env python
'''
This script reads a URDF file and copies all required mesh files to <path_to_pkg>/meshes/. This is necessary, because
Mujoco requires to find all meshes in a single folder.
'''

import os
import shutil
from urdf_parser_py.urdf import URDF, Link, Mesh, Visual, Collision
import argparse
import rospkg
import trimesh
import subprocess

def main(urdf_file):
    proc = subprocess.Popen(['./run_xacro.sh'])
    proc.wait()
    robot = URDF.from_xml_file(os.path.join(os.path.dirname(__file__),'../world_model/kinova_fel_with_objects.urdf'))
    assert isinstance(robot, URDF)

    r = rospkg.RosPack()
    ros_root = rospkg.get_ros_root()
    # path = r.get_path('rospy')


    meshes = []
    for l in robot.links:
        assert isinstance(l, Link)
        for visual in l.visuals + l.collisions: # type: Visual
            if isinstance(visual.geometry, Mesh):
                meshes.append(visual.geometry.filename)

    meshes = list(set(meshes))

    dst_path = os.path.join(os.path.dirname(__file__), "../meshes")

    for path in meshes:
        assert (isinstance(path, str))
        rel_path = path.replace('package://', '', 1)
        pkg_name = rel_path.split('/')[0]
        rel_path = rel_path.replace(pkg_name, '', 1)
        pkg_path = r.get_path(pkg_name)
        mesh_path_abs = os.path.join(pkg_path, rel_path.replace('/', '', 1))
        shutil.copy(mesh_path_abs, dst_path)

        mesh_name = mesh_path_abs.split('/')[-1]
        if '.dae' in mesh_name:
            dst_file_path = os.path.join(dst_path, mesh_name.replace('.dae', '.STL'))
            mesh = trimesh.load(mesh_path_abs)
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.geometry.values()[0]
            mesh.export(file_obj=dst_file_path, file_type='stl')



    # for l in robot.links:
    #     assert isinstance(l, Link)
    #     for visual in l.visuals + l.collisions: # type: Visual
    #         if isinstance(visual.geometry, Mesh) and '.dae' in visual.geometry.filename:
    #             assert isinstance(visual.geometry.filename, str)
    #             filename = visual.geometry.filename.replace('.dae', '.stl')
    #             visual.geometry.filename = filename

    # robot = URDF.from_xml_file(os.path.join(os.path.dirname(__file__),'../world_model/kinova_fel_with_objects.urdf'))
    file_object = open(os.path.join(os.path.dirname(__file__),'../world_model/kinova_fel_with_objects.urdf'), 'r+')
    lines = file_object.readlines()
    file_object.close()
    res_str = "package://kinova_mujoco/"
    # '        <mesh filename="package://kortex_description/arms/gen3/7dof/meshes/base_link.STL"/>'
    new_lines = []
    for line in lines:
        # take over comment lines unchanged
        if '<!--' in line:
            new_lines.append(line + "\n")
        elif '<mesh filename="package://' in line:
            # write new stl mesh location in robot mujoco package
            link_name = line.split('/')[-2]
            if 'scale' in link_name:
                pass
                # link_name.
            new_line = line.split('//')[0] + '//' + 'kinova_mujoco/meshes/' + link_name.replace('.dae', '.STL') + '/>'
            # line = line.replace('.dae', '.stl')
            new_lines.append(new_line + "\n")
        elif '<material name="">' in line:
            # mujoco wants everything to have a filled material tag
            new_line = line.replace('""', '"DUMMY_MATERIAL"')
            new_lines.append(new_line + "\n")
        elif '<mimic joint=' in line:
            # mujoco does not support mimic joints. we have to use a custom controller to make the mimic functionality.
            pass
        else:
            # take over normal lines
            new_lines.append(line + "\n")


    file_object = open(os.path.join(os.path.dirname(__file__),'../world_model/kinova_fel_with_objects_NoDae.urdf'), 'w+')
    file_object.writelines(new_lines)
    file_object.close()
    print("robot")


if __name__ == "__main__":
    my_parser = argparse.ArgumentParser(description='Collect all meshes for a URDF.')

    my_parser.add_argument('Path',
                           metavar='path',
                           type=str,
                           help='the path to URDF',
                           default='')

    # args = my_parser.parse_args()
    # input_path = args.Path

    # main(input_path)
    main('test')