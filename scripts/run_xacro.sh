#!/usr/bin/env bash
kortex_description='rospack find kinova_mujoco'
folder=`$kortex_description`
file="${folder}/world_model/kinova_fel_with_objects.xacro"
rosrun xacro xacro --inorder $file -o "${folder}/world_model/kinova_fel_with_objects.urdf"
