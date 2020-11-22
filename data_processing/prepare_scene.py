
from v2a_convert_utils import create_object_scene
from action_classes import V2aScene, myObject

scene_file = 'full_table_scene/scenes_V2A.json'
scene_id = 0


scene = V2aScene.load_scene(path=scene_file, scene_id=scene_id)
create_object_scene(scene.objects, scene_id)