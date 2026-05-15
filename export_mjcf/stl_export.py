# This file is a copy paste of the old (pre 4.2) Blender STL Export
# The old STL exporter is required for the global_space argument.

# The original file had the following copyright information:
# SPDX-FileCopyrightText: 2010-2022 Blender Foundation
# SPDX-License-Identifier: GPL-2.0-or-later
# Author: Guillaume Bouchard (Guillaum)

import bpy
from bpy.props import (
    StringProperty,
    BoolProperty,
    CollectionProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
)
from bpy_extras.io_utils import (
    ExportHelper,
    orientation_helper,
    axis_conversion,
)
from bpy.types import (
    Operator,
    OperatorFileListElement,
)


# The Original Utilities

def faces_from_mesh(ob, global_matrix, use_mesh_modifiers=False):
    """
    From an object, return a generator over a list of faces.

    Each faces is a list of his vertices. Each vertex is a tuple of
    his coordinate.

    use_mesh_modifiers
        Apply the preview modifier to the returned liste

    triangulate
        Split the quad into two triangles
    """

    import bpy

    # get the editmode data
    if ob.mode == "EDIT":
        ob.update_from_editmode()

    # get the modifiers
    if use_mesh_modifiers:
        depsgraph = bpy.context.evaluated_depsgraph_get()
        mesh_owner = ob.evaluated_get(depsgraph)
    else:
        mesh_owner = ob

    # Object.to_mesh() is not guaranteed to return a mesh.
    try:
        mesh = mesh_owner.to_mesh()
    except RuntimeError:
        return

    if mesh is None:
        return

    mat = global_matrix @ ob.matrix_world
    mesh.transform(mat)
    if mat.is_negative:
        mesh.flip_normals()
    mesh.calc_loop_triangles()

    vertices = mesh.vertices

    for tri in mesh.loop_triangles:
        yield [vertices[index].co.copy() for index in tri.vertices]

    mesh_owner.to_mesh_clear()


# an stl binary file is
# - 80 bytes of description
# - 4 bytes of size (unsigned int)
# - size triangles :
#
#   - 12 bytes of normal
#   - 9 * 4 bytes of coordinate (3*3 floats)
#   - 2 bytes of garbage (usually 0)
BINARY_HEADER = 80
BINARY_STRIDE = 12 * 4 + 2


def _header_version():
    import bpy
    return "Exported from Blender-" + bpy.app.version_string


def _binary_write(filepath, faces):
    import struct
    import itertools
    from mathutils.geometry import normal

    with open(filepath, 'wb') as data:
        fw = data.write
        # header
        # we write padding at header beginning to avoid to
        # call len(list(faces)) which may be expensive
        fw(struct.calcsize('<80sI') * b'\0')

        # 3 vertex == 9f
        pack = struct.Struct('<9f').pack

        # number of vertices written
        nb = 0

        for face in faces:
            # calculate face normal
            # write normal + vertices + pad as attributes
            fw(struct.pack('<3f', *normal(*face)) + pack(*itertools.chain.from_iterable(face)))
            # attribute byte count (unused)
            fw(b'\0\0')
            nb += 1

        # header, with correct value now
        data.seek(0)
        fw(struct.pack('<80sI', _header_version().encode('ascii'), nb))


def _ascii_write(filepath, faces):
    from mathutils.geometry import normal

    with open(filepath, 'w') as data:
        fw = data.write
        header = _header_version()
        fw('solid %s\n' % header)

        for face in faces:
            # calculate face normal
            fw('facet normal %f %f %f\nouter loop\n' % normal(*face)[:])
            for vert in face:
                fw('vertex %f %f %f\n' % vert[:])
            fw('endloop\nendfacet\n')

        fw('endsolid %s\n' % header)


def write_stl(filepath="", faces=(), ascii=False):
    """
    Write a stl file from faces,

    filepath
       output filepath

    faces
       iterable of tuple of 3 vertex, vertex is tuple of 3 coordinates as float

    ascii
       save the file in ascii format (very huge)
    """
    (_ascii_write if ascii else _binary_write)(filepath, faces)

    
    
# The Export Itself

@orientation_helper(axis_forward='Y', axis_up='Z')
class ExportSTL_Legacy(Operator, ExportHelper):
    bl_idname = "export_mesh.stl_legacy"
    bl_label = "Export STL Legacy"
    bl_description = """Save STL triangle mesh data. The legacy (pre 4.2) exporter."""

    filename_ext = ".stl"
    filter_glob: StringProperty(default="*.stl", options={'HIDDEN'})

    use_selection: BoolProperty(
        name="Selection Only",
        description="Export selected objects only",
        default=False,
    )
    global_scale: FloatProperty(
        name="Scale",
        min=0.01, max=1000.0,
        default=1.0,
    )
    use_scene_unit: BoolProperty(
        name="Scene Unit",
        description="Apply current scene's unit (as defined by unit scale) to exported data",
        default=False,
    )
    ascii: BoolProperty(
        name="Ascii",
        description="Save the file in ASCII file format",
        default=False,
    )
    use_mesh_modifiers: BoolProperty(
        name="Apply Modifiers",
        description="Apply the modifiers before saving",
        default=True,
    )
    batch_mode: EnumProperty(
        name="Batch Mode",
        items=(
            ('OFF', "Off", "All data in one file"),
            ('OBJECT', "Object", "Each object as a file"),
        ),
    )
    global_space: FloatVectorProperty(
        name="Global Space",
        description="Export in this reference space",
        subtype='MATRIX',
        size=(4, 4),
    )

    @property
    def check_extension(self):
        return self.batch_mode == 'OFF'

    def execute(self, context):
        import os
        import itertools
        from mathutils import Matrix

        keywords = self.as_keywords(
            ignore=(
                "axis_forward",
                "axis_up",
                "use_selection",
                "global_scale",
                "check_existing",
                "filter_glob",
                "use_scene_unit",
                "use_mesh_modifiers",
                "batch_mode",
                "global_space",
            ),
        )

        scene = context.scene
        if self.use_selection:
            data_seq = context.selected_objects
        else:
            data_seq = scene.objects

        # Take into account scene's unit scale, so that 1 inch in Blender gives 1 inch elsewhere! See T42000.
        global_scale = self.global_scale
        if scene.unit_settings.system != 'NONE' and self.use_scene_unit:
            global_scale *= scene.unit_settings.scale_length

        global_matrix = axis_conversion(
            to_forward=self.axis_forward,
            to_up=self.axis_up,
        ).to_4x4() @ Matrix.Scale(global_scale, 4)

        if self.properties.is_property_set("global_space"):
            global_matrix = global_matrix @ self.global_space.inverted()

        if self.batch_mode == 'OFF':
            faces = itertools.chain.from_iterable(
                    faces_from_mesh(ob, global_matrix, self.use_mesh_modifiers)
                    for ob in data_seq)

            write_stl(faces=faces, **keywords)
        elif self.batch_mode == 'OBJECT':
            prefix = os.path.splitext(self.filepath)[0]
            keywords_temp = keywords.copy()
            for ob in data_seq:
                faces = faces_from_mesh(ob, global_matrix, self.use_mesh_modifiers)
                keywords_temp["filepath"] = prefix + bpy.path.clean_name(ob.name) + ".stl"
                write_stl(faces=faces, **keywords_temp)

        return {'FINISHED'}

    def draw(self, context):
        pass


def register():
    bpy.utils.register_class(ExportSTL_Legacy)

    # NOTE: Unlike the original, this does not register the STL exporter
    #       to the File > Export Menu; the old functionality is used only
    #       within this addon and users exporting will use the latest version.


def unregister():
    bpy.utils.unregister_class(ExportSTL_Legacy)


if __name__ == "__main__":
    register()
