
# Kinova MuJoCo simulation with scene of objects

![Image of Yaktocat](scripts/simstate.png)

Start simulation by 

```shell script
roslaunch kinova_mujoco table_simulation_generated.launch
```

## Scene preparation for MuJoCo

MuJoCo requires the whole scene to be described by a single model.
We generate URDFs from the robot workspace plus objects. To 
make the simulation more stable, we decompose the object meshes
into mant small convex bodies. 

A sample scene description is given in `data_processing/full_table_scene`
(A [SHOP VRB scene](https://michaal94.github.io/SHOP-VRB/)). 
The scenes can be loaded from compatible json files using `prepare_scene.py`
or created in python code. 

MuJoCo requires that all meshes for the simulation are placed in a single 
folder. Therefore, we copy all meshes referenced in our URDF to the folder 
[meshes](meshes) using the script `scripts/collect_meshes.py`.


