print("run script")

import bpy

import gpu
shader = gpu.shader.from_builtin('3D_SMOOTH_COLOR')
from gpu_extras.batch import batch_for_shader

draw_handler = None

from mathutils import Color
color_increment = 0.37

def draw(scene):
    vertices = []
    colors = []
        
    color = Color()
    color.hsv = (0, 1.0, 1.0)
    # go through the bodies..
    for obj in scene.objects:
        for i, child in obj['tf_tree_children'].items():
            vertices.append(child.matrix_world.translation)
            vertices.append(obj.matrix_world.translation)
            colors.append( color[:] + (1.0,))
            colors.append( color[:] + (1.0,))
            color.h = (color.h + color_increment) % 1.0

    batch = batch_for_shader(shader, 'LINES', {"pos": vertices, "color": colors})
    shader.bind()
    batch.draw(shader)

def update(scene):
    #if draw_handler is not None:
    #    bpy.types.SpaceView3D.draw_handler_remove(draw_handler, 'WINDOW')
    draw_handler = bpy.types.SpaceView3D.draw_handler_add(draw, (scene, ), 'WINDOW', 'POST_VIEW')
    print("update call")
    # TODO be able to remove these? or not have them run constantly?


bpy.app.handlers.depsgraph_update_pre.append(update)
