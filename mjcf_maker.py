# Simple MJCF Kinematics Exporter for Blender
# Jay J.
# 2020

### docs; model structure
# any mesh, any shape
# name them nicely (link_xx)

# then create empties (prefer arrows type)
# empty sets the location and orientation of joints between bodies
# name it nicely (joint_xx)
# add a (custom) constraint type to the empty
# this lets you select the joint type, parent, and child body

# joint can only have one parent link and child link
# links can have unlimited related joints

# spits out the .sdf file in the current (blender) directory
# spits out the .stl files in a subdirectory mesh_stl, will create this if it needs to

import bpy
import time
from math import pi
from mathutils import Vector
import os
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.dom import minidom
# https://pymotw.com/2/xml/etree/ElementTree/create.html

def xml_pretty(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="    ")

def export_pretty(context, xml_root):
    # make it user-readable and atually do the file export
    xml_pretty_string = xml_pretty(xml_root)
    print(xml_pretty_string)
    fd = open(bpy.path.abspath('//' + context.scene.robot_name + '.xml'), 'w')
    fd.write(xml_pretty_string)
    fd.close()

# explores all objects in context.scene.objects to 
# tells bodies which joints are DOWNSTREAM of them
# tells bodies which geoms are DOWNSTREAM of them
# tells bodies which bodies are DOWNSTREAM of them
# tells joints which joints are DOWNSTREAM of them
# bodies must be the children of bodies, not the children of joints here

def build_kinematic_tree(context):
    for obj in context.scene.objects:
        obj['sk_child_entity_list'] = {}
    
    for obj in context.scene.objects:
        if not( obj.sk_parent_entity == None):

            parent = obj.sk_parent_entity
            if obj.enum_sk_type == "body":
                # search for the upper body
                attempts = 0
                attempt_limit = 512

                while parent.enum_sk_type != "body":
                    parent = parent.sk_parent_entity
                    attempts += 1
                    assert(attempts < attempt_limit)

            # now register with the parent
            dict_tmp = context.scene.objects[parent.name]['sk_child_entity_list']
            dict_tmp[repr(len(dict_tmp.keys())+1)] = obj

def export_stl(context, obj):
    print(f"Exporting STL for object {obj.name}")
    # store the active object so it can be restored later
    sel_obj = bpy.context.view_layer.objects.active
    
    # TODO find a way for collection instances to serve as single links
    if obj.type != 'MESH':
        print("ERROR object", obj.name, "is not a mesh object")
        return
    
    # select only the single mesh to be exported
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    
    # CAUTION! the option use_global_frame requres modifying Blender D11517 (https://developer.blender.org/D11517)
    bpy.ops.export_mesh.stl(filepath=bpy.path.abspath('//mesh_stl/' + obj.name + '.stl'), use_selection=True, use_global_frame=False, ascii=False)
    
    # restore selection
    bpy.ops.object.select_all(action='DESELECT')
    sel_obj.select_set(True)
    
def export_link_stl_collision(context, obj, xml_link):
    xml_collision = SubElement(xml_link, "collision")
    xml_collision.set("name", obj.name+"_collision")

    # change to collision object if one is defined    
    obj_collision = obj

    xml_pose(context, obj_collision, xml_collision)

    xml_geometry = SubElement(xml_collision, "geometry")
    export_stl(context, obj, xml_geometry)
 
def export_link_stl_visual(context, obj, xml_link):   
    # get the pose
    xml_visual = SubElement(xml_link, 'visual')
    xml_visual.set('name', obj.name + '_visual')
    xml_geometry = SubElement(xml_visual, 'geometry')
    
    xml_pose(context, obj, xml_visual)

    export_stl(context, obj, xml_geometry)

def export_options(xml_root): # TODO expose these in some sort of UI panel
    xml_compiler = SubElement(xml_root, "compiler")
    xml_compiler.set("angle", "radian")

    xml_options = SubElement(xml_root, "option")
    xml_options.set("timestep", "0.001")
    xml_options.set("iterations", "20")

    xml_sensornoise = SubElement(xml_options, "flag")
    xml_sensornoise.set("sensornoise", "enable")

def export_setup_folders():
    if not os.path.exists(bpy.path.abspath('//mesh_stl')):
        print('Did not find mesh folder, creating it')
        os.makedirs(bpy.path.abspath('//mesh_stl'))

def export_defaults(): # TODO
    pass

def export_mesh_geom(context, obj, xml, xml_asset):
    xml_geom = SubElement(xml, "geom")
    xml_geom.set("name", obj.name)
    xml_geom.set("type", "mesh")
    xml_geom.set("pos", "0 0 0")
    xml_geom.set("quat", "1 0 0 0")
    xml_geom.set("mass", repr(obj.sk_mass))
    xml_geom.set("mesh", "mesh_" + obj.name)

    xml_stl = SubElement(xml_asset, "mesh")
    xml_stl.set("name", "mesh_" + obj.name)
    xml_stl.set("file", "mesh_stl/" + obj.name + ".stl")
    export_stl(context, obj)

# exports xml data for a link entity
def export_entity(context, obj, xml_model, body_is_root, xml_asset):

    if not body_is_root:
        tf = obj.sk_parent_entity.matrix_world.inverted_safe() @ obj.matrix_world
        pos = repr(tf.translation[0]) + " " + repr(tf.translation[1]) + " " + repr(tf.translation[2])

        # TODO verify orientation quaternion
        # relative to the parent?
        q = tf.to_quaternion()
        quat = repr(q[0]) + " " + repr(q[1]) + " " + repr(q[2]) + " " + repr(q[3])

    if obj.enum_sk_type == "geom":
        print(f"Exporting {obj.name} as GEOM")
        xml_entity = SubElement(xml_model, "geom")
        xml_entity.set("name", obj.name)
        xml_entity.set("type", obj.sk_geom_type)
        xml_entity.set("size", calculate_geom_size_str(obj) )

        xml_entity.set("pos", pos)
        xml_entity.set("quat", quat)
        xml_entity.set("mass", repr(obj.sk_mass))

    elif obj.enum_sk_type == "joint":
        print(f"Exporting {obj.name} as JOINT")
        # TODO how to handle multiple joints attached to the same body?
        # export_joint(context, joint, xml_entity, False)
        xml_entity = SubElement(xml_model, "joint")
        xml_entity.set("type", obj.enum_joint_type)
        xml_entity.set("pos", pos)

        axis_dict = {"x":Vector([1,0,0]), "y":Vector([0,1,0]), "z":Vector([0,0,1]), "":Vector([0,0,0])}
        axis_local = axis_dict[obj.enum_joint_axis]
        axis_global = obj.matrix_world.to_3x3() @ axis_local
        axis_parent = obj.sk_parent_entity.matrix_world.inverted_safe().to_3x3() @ axis_global

        xml_entity.set("axis", repr(axis_parent[0]) + " " + repr(axis_parent[1]) + " " + repr(axis_parent[2]))

        # TODO stiffness damping, other properties

        if obj.enum_joint_type != "free":
            xml_entity.set("limited", repr(obj.sk_axis_limit).lower())
            if obj.sk_axis_limit:
                if obj.enum_joint_type == "slide":
                    xml_entity.set("range", repr(obj.sk_axis_lower_lin) + " " + repr(obj.sk_axis_upper_lin))
                elif obj.enum_joint_type == "ball":
                    xml_entity.set("range", "0 " + repr(obj.sk_axis_upper_rot))
                else:
                    xml_entity.set("range", repr(obj.sk_axis_lower_rot) + " " + repr(obj.sk_axis_upper_rot))

        for j, child in obj['sk_child_entity_list'].iteritems():
            print("calling child: ", child)
            export_entity(context, child, xml_model, False, xml_asset)

    elif obj.enum_sk_type == "body":
        print(f"Exporting {obj.name} as BODY")

        if body_is_root:
            # just worldbody with no info
            # this is fed in from up above
            # TODO how to bring in better the world geometry body?
            xml_entity = xml_model
        else:
            xml_entity = SubElement(xml_model, 'body')
            xml_entity.set("name", obj.name)

            xml_entity.set("pos", pos)
            xml_entity.set("quat", quat)
            
            if not obj.sk_use_primitive_geom:
                export_mesh_geom(context, obj, xml_entity, xml_asset)

        for j, child in obj['sk_child_entity_list'].iteritems():
            print("calling child: ", child)
            export_entity(context, child, xml_entity, False, xml_asset)

    else:
        print(f"[ERROR] something has gone wrong exporting {obj.name}!")


    # how to handle joints that are in parallel?

# follows an already-explored tree to add links and joints to the xml data    
def export_tree(context):
    root = context.object
    export_start_time = time.time()
    
    export_setup_folders()
    
    xml_root = Element('mujoco')
    xml_root.set('model', context.scene.robot_name)

    export_options(xml_root)
    export_defaults()

    xml_asset = SubElement(xml_root, "asset")

    xml_worldbody = SubElement(xml_root, "worldbody")
    
    # TODO search the scene for root object(s) - and static ground geometries
    # maybe have a world/ground empty? that can consistently start everything, have things referenced to it!

    # explore the kinematic tree from root down
    export_entity(context, root, xml_worldbody, True, xml_asset)

    # TODO lights and cameras inside worldbody
    xml_light_tmp = SubElement(xml_worldbody, "light")
    xml_light_tmp.set("directional", "true")
    xml_light_tmp.set("cutoff", "4")
    xml_light_tmp.set("exponent", "20")
    xml_light_tmp.set("diffuse", "1 1 1")
    xml_light_tmp.set("specular", "0 0 0")
    xml_light_tmp.set("pos", "0.9 0.3 2.5")
    xml_light_tmp.set("dir", "-0.9 -0.3 -2.5")
                
    xml_actuator = SubElement(xml_root, "actuator") # TODO
    xml_equality = SubElement(xml_root, "equality") # TODO
    xml_sensor = SubElement(xml_root, "sensor") # TODO

    export_pretty(context, xml_root)
    
# this class is responsible for responding to the 'export' button being pushed
class MJCFExportOperator(bpy.types.Operator):
    bl_idname = 'sk.export_mjcf'
    bl_label = 'Export MJCF'
    bl_description = 'Export MJCF'
    bl_options = {'REGISTER', 'UNDO'}
    
    action: bpy.props.EnumProperty(
        items=[('MJCF', 'mjcf', 'mjcf')]
        )
    
    def execute(self, context):
        print('Export action called, root object=', context.object)
        build_kinematic_tree(context)
        export_tree(context)
        print('export, successful')
        
        return {'FINISHED'}

def calculate_geom_size(obj):
    size = None
    if obj.sk_geom_type == "plane":
        # x half; y half; square grid spacing
        size = [abs(obj.scale[0]), abs(obj.scale[1]), abs(obj.scale[2])]

    if obj.sk_geom_type == "sphere":
        # radius
        size = [abs(obj.scale[0])]

    if obj.sk_geom_type == "capsule":
        # radius, center length
        size = [abs(obj.scale[0]), abs(obj.scale[2])]

    if obj.sk_geom_type == "ellipsoid":
        # radius ; radius ; radius
        size = [abs(obj.scale[0]), abs(obj.scale[1]), abs(obj.scale[2])]

    if obj.sk_geom_type == "cylinder":
        # radius; length
        size = [abs(obj.scale[1]), abs(2*obj.scale[0])]

    if obj.sk_geom_type == "box":
        # x y z
        size = [abs(obj.scale[0]), abs(obj.scale[1]), abs(obj.scale[2])]    

    return size

def calculate_geom_size_str(obj):
    s = ""
    for a in calculate_geom_size(obj):
        s += repr(a)+" "
    return s

# this is the panel in the constraints window where you define joint information
class SimpleKinematicsJointPanel(bpy.types.Panel):
    """Creates a Panel in the Constraints properties window"""
    bl_label = "Simple Kinematics"
    bl_idname = "OBJECT_PT_simplekinematics"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "constraint"

    def draw(self, context):
        layout = self.layout
        obj = context.object

        row = layout.row()
        row.prop(context.scene, "robot_name", text="Robot Name")
        
        row = layout.row()
        row.operator('sk.export_mjcf', text="Export using this object as root").action = 'MJCF'

        row = layout.row()
        row.prop(obj, 'enum_sk_type', text='Type', expand=True)
        
        if obj.enum_sk_type == 'body':

            # TODO if body is to built of geoms, then those should drive the mass

            row = layout.row()
            row.prop(obj, "sk_parent_entity", text="Parent Body")

            row = layout.row()
            row.prop(obj, "sk_use_primitive_geom", text="Use geometry primitives")

            if obj.sk_use_primitive_geom:
                row = layout.row()
                row.label(text="Derived Mass (kg): TODO")

            else:
                row = layout.row()
                row.prop(obj, 'sk_mass', text='Mass (kg)')

        if obj.enum_sk_type == "geom":
            if obj.type != "EMPTY":
                row = layout.row()
                row.label(text="ERROR! You must represent geometry primitives with empty-type objects.")
            else:
                row = layout.row()
                row.prop(obj, 'sk_mass', text='Mass (kg)')

                row = layout.row()
                row.prop(obj, "sk_parent_entity", text="Parent Body")

                row = layout.row()
                row.prop(obj, 'sk_geom_type', text='Type')

                if obj.sk_geom_type == 'box':
                    size = calculate_geom_size(obj)
                    row = layout.row()
                    row.label(text=f"Size: {size[0]:.3f} {size[1]:.3f} {size[2]:.3f}m")
                elif obj.sk_geom_type == 'sphere':
                    size = calculate_geom_size(obj)
                    row = layout.row()
                    row.label(text=f"Radius: {size[0]:.3f}m")
                else:
                    size = calculate_geom_size(obj)
                    row = layout.row()
                    row.label(text=f"raw size array: {size}")

        if obj.enum_sk_type == 'joint':
                        
            row = layout.row()
            row.prop(obj, "sk_parent_entity", text="Child BODY or Parent JOINT")

            row = layout.row()
            row.label(text="Joint Properties")
                   
            row = layout.row()
            row.prop(obj, 'enum_joint_type', text='joint type', expand=True)

            # TODO stiffness, damping, etc.
            
            if obj.enum_joint_type == 'hinge' or obj.enum_joint_type == 'slide' or obj.enum_joint_type == 'ball':
                                    
                # a block for basic axis settings
                row = layout.row()
                row.prop(obj, 'enum_joint_axis', text='Axis')

                row = layout.row()
                row.prop(obj, 'sk_axis_limit', text='limit')

                if obj.sk_axis_limit:
                    if obj.enum_joint_type == 'slide':
                        row.prop(obj, 'sk_axis_lower_lin', text='lower limit')
                        row.prop(obj, 'sk_axis_upper_lin', text='upper limit')
                    elif obj.enum_joint_type == 'hinge':
                        row.prop(obj, 'sk_axis_lower_rot', text='lower limit')
                        row.prop(obj, 'sk_axis_upper_rot', text='upper limit')
                    else:
                        row.prop(obj, 'sk_axis_upper_rot', text='upper limit')

            row = layout.row()
            row.prop(obj, "sk_is_actuator")

            if obj.sk_is_actuator:
                row = layout.row()
                row.label(text="TODO add some actuator properties!")
                row = layout.row()
                row.label(text="type, effort limits, reflected inertia...")

enum_joint_type_options = [
    ('free', 'Free', '', 1), # rules!
    ('ball', 'Ball', '', 2), # rules! body cannot have other rotary joints but can have slide
    ('hinge', 'Hinge', '', 3),
    ('slide', 'Slide', '', 4)
    ]
    
enum_joint_axis_options = [
    ('x', 'X', 'x axis'),
    ('y', 'Y', 'y axis'),
    ('z', 'Z', 'z axis')
    ]
    
enum_sk_type = [
    ('body', 'body', 'body'),
    ('joint', 'joint', 'joint'),
    ('geom', 'geom', 'geom')
    ]
    
enum_geom_type_options = [
    ('plane', 'plane', 'plane'),
    ('hfield', 'hfield', 'hfield'),
    ('sphere', 'sphere', 'sphere'),
    ('capsule', 'capsule', 'capsule'),
    ('ellipsoid', 'ellipsoid', 'ellipsoid'),
    ('cylinder', 'cylinder', 'cylinder'),
    ('box', 'box', 'box'),
    ('mesh', 'mesh', 'mesh')
    ]

def register():
    # step angle for UI drags
    step_angle_ui = 1500 # about 15 degrees
    step_lin_ui = 10 # 0.1m
    
    # General Properties
    bpy.types.Scene.robot_name = bpy.props.StringProperty(name="robot_name", default="")
    bpy.types.Object.enum_sk_type = bpy.props.EnumProperty(items=enum_sk_type)
    bpy.types.Object.sk_parent_entity = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_parent_entity", description="Simple Kinematics Parent Entity", update=None)

    # Body & Geom Properties
    bpy.types.Object.sk_mass = bpy.props.FloatProperty(name='sk_mass', default=1)
    bpy.types.Object.sk_use_primitive_geom = bpy.props.BoolProperty(name="use_primitive_geom", default=True)
    bpy.types.Object.sk_geom_type = bpy.props.EnumProperty(items=enum_geom_type_options)

    # Joint Properties
    bpy.types.Object.enum_joint_type = bpy.props.EnumProperty(items=enum_joint_type_options)
    bpy.types.Object.enum_joint_axis = bpy.props.EnumProperty(items=enum_joint_axis_options, default="x")

    bpy.types.Object.sk_axis_limit = bpy.props.BoolProperty(name="sk_axis_limit", default=False)
    bpy.types.Object.sk_axis_lower_rot = bpy.props.FloatProperty(name="lower_rot", default=0, soft_min=-2*pi, soft_max=2*pi, unit="ROTATION", step=step_angle_ui)
    bpy.types.Object.sk_axis_upper_rot = bpy.props.FloatProperty(name="upper_rot", default=0, soft_min=-2*pi, soft_max=2*pi, unit="ROTATION", step=step_angle_ui)
    bpy.types.Object.sk_axis_lower_lin = bpy.props.FloatProperty(name="lower_lin", default=0, soft_min=-1, soft_max=1, unit="LENGTH", step=step_lin_ui)
    bpy.types.Object.sk_axis_upper_lin = bpy.props.FloatProperty(name="upper_lin", default=0, soft_min=-1, soft_max=1, unit="LENGTH", step=step_lin_ui)

    # Actuator Properties
    bpy.types.Object.sk_is_actuator = bpy.props.BoolProperty(name="is_actuator", default=False)        

    bpy.utils.register_class(MJCFExportOperator)
    bpy.utils.register_class(SimpleKinematicsJointPanel)

def unregister():
    bpy.utils.unregister_class(MJCFExportOperator)
    bpy.utils.unregister_class(SimpleKinematicsJointPanel)
    del bpy.types.Scene.robot_name
    del bpy.types.Object.enum_joint_type
    del bpy.types.Object.enum_joint_axis
    del bpy.types.Object.sk_parent_entity
    del bpy.types.Object.sk_child_entity
    del bpy.types.Object.enum_sk_type
    del bpy.types.Object.sk_mass
    del bpy.types.Object.sk_is_actuator

if __name__ == "__main__":
    register()
    print('register, successful')

