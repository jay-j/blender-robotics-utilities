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
    return reparsed.toprettyxml(indent="   ")

# explores all objects in context.scene.objects to 
# tells bodies which joints are DOWNSTREAM of them
# tells bodies which geoms are DOWNSTREAM of them
# bodies can have multiple child geoms
# bodies can have ONE child joint
# joints can have ONE child entity
def build_kinematic_tree(context):
    for obj in context.scene.objects:
        obj['sk_child_entity_list'] = {}
    
    for obj in context.scene.objects:
        if not( obj.sk_parent_entity == None):
            dict_tmp = context.scene.objects[obj.sk_parent_entity.name]['sk_child_entity_list']
            dict_tmp[repr(len(dict_tmp.keys())+1)] = obj
            # print("Dictionary!", dict_tmp)

def inertia_of_box(obj):
    inertia = {}
    inertia['ixx'] = (1.0/12.0)*obj.sk_mass*(obj.dimensions[1]**2 + obj.dimensions[2]**2)
    inertia['iyy'] = (1.0/12.0)*obj.sk_mass*(obj.dimensions[0]**2 + obj.dimensions[2]**2)
    inertia['izz'] = (1.0/12.0)*obj.sk_mass*(obj.dimensions[0]**2 + obj.dimensions[1]**2)
    inertia['ixy'] = 0
    inertia['ixz'] = 0
    inertia['iyz'] = 0
    return inertia

def export_link_inertial(context, obj, xml_link):
    xml_mass = SubElement(xml_link, 'mass')
    xml_mass.text = repr(obj.sk_mass)
    
    xml_inertia = SubElement(xml_link, 'inertia')
    inertia = inertia_of_box(obj)
    for i in inertia:
        xml_i = SubElement(xml_inertia, i)
        xml_i.text = repr(inertia[i])

def xml_pose(context, obj, xml_entity):
    xform = obj.matrix_world.inverted()
    pose = ''
    pose += repr(xform.translation[0]) + " "
    pose += repr(xform.translation[1]) + " "
    pose += repr(xform.translation[2]) + " "
    rot = xform.to_euler('XYZ')
    pose += repr(rot.x) + " "
    pose += repr(rot.y) + " "
    pose += repr(rot.z)
    
    xml_pose = SubElement(xml_entity, 'pose')
    xml_pose.text = pose


def export_stl(context, obj, xml_geometry):
    # store the active object so it can be restored later
    sel_obj = bpy.context.view_layer.objects.active
    
    # TODO find a way for collection instances to serve as single links
    if obj.type != 'MESH':
        print("ERROR object", obj.name, "is not a mesh object")
        return
    
    # select only the single mesh to be exported
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    
    # TODO use_global_frame=False
    bpy.ops.export_mesh.stl(filepath=bpy.path.abspath('//mesh_stl/' + obj.name + '.stl'), use_selection=True)
    
    # restore selection
    bpy.ops.object.select_all(action='DESELECT')
    sel_obj.select_set(True)
    
    xml_mesh = SubElement(xml_geometry, "mesh")
    xml_uri = SubElement(xml_mesh, "uri")
    xml_uri.text = 'file://'+ bpy.path.abspath('//mesh_stl/' + obj.name + '.stl')
    
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



# exports xml data for a joint entity  
def export_joint(context, obj, xml_model):
    print("exporting joint", obj.name)
    xml_joint = SubElement(xml_model, 'joint')
    xml_joint.set('name', obj.name)
    
    xml_joint.set('type',  obj.enum_joint_type)
    
    xml_joint_parent = SubElement(xml_joint, 'parent')
    xml_joint_parent.text = obj.sk_joint_parent.name
    xml_joint_child = SubElement(xml_joint, 'child')
    xml_joint_child.text = obj.sk_joint_child.name
    
    # pose: where the joint is relative to the child frame, in the child frame
    # x y z angle angle angle (Euler roll pitch yaw; extrinsic x-y-z rotation)
    xml_joint_pose = SubElement(xml_joint, 'pose')
    pose_wrt_child = obj.sk_joint_child.matrix_world.inverted() @ obj.matrix_world # the '@' notation means matrix (not element) multiplication

    pose = ""
    pose += repr(pose_wrt_child.translation[0]) + " "
    pose += repr(pose_wrt_child.translation[1]) + " "
    pose += repr(pose_wrt_child.translation[2]) + " "
    
    rot = pose_wrt_child.to_euler('XYZ')
    pose += repr(rot.x) + " "
    pose += repr(rot.y) + " "
    pose += repr(rot.z)
    
    xml_joint_pose.text = pose
        
    # axis 1
    if ['revolute', 'continuous', 'prismatic', 'universal'].count(joint_subtype) > 0:
        # build nicer arrays of axis properties, with 0,1,2 as xyz
        axis_enabled = [obj.sk_axis_x_enabled, obj.sk_axis_y_enabled, obj.sk_axis_z_enabled]
        axis_limit = [obj.sk_axis_x_limit, obj.sk_axis_y_limit, obj.sk_axis_z_limit]
        if ['prismatic'].count(joint_subtype) > 0:
            axis_lower = [obj.sk_axis_x_lower_lin, obj.sk_axis_y_lower_lin, obj.sk_axis_z_lower_lin]
            axis_upper = [obj.sk_axis_x_upper_lin, obj.sk_axis_y_upper_lin, obj.sk_axis_z_upper_lin]
        else:
            axis_lower = [obj.sk_axis_x_lower_rot, obj.sk_axis_y_lower_rot, obj.sk_axis_z_lower_rot]
            axis_upper = [obj.sk_axis_x_upper_rot, obj.sk_axis_y_upper_rot, obj.sk_axis_z_upper_rot]
        
        # figure out which axis is to be axis1
        axis1 = -1
        for i in range(0,3):
            if axis_enabled[i]:
                axis1 = i
                break
        print("found axis: ", axis1)
        
        # axis unit vector expressed in the joint frame
        axis_dict = {0:'1 0 0', 1:'0 1 0', 2:'0 0 1'}
        xml_axis = SubElement(xml_joint, 'axis')
        xml_xyz = SubElement(xml_axis, 'xyz')
        xml_xyz.text = axis_dict[axis1]
        
        # set axis limits for those that allow it
        if ['revolute', 'prismatic'].count(joint_subtype) > 0:
            if axis_limit[axis1]:
                xml_limit = SubElement(xml_axis, 'limit')
                xml_lower = SubElement(xml_limit, 'lower')
                xml_lower.text = repr(axis_lower[axis1])
                xml_upper = SubElement(xml_limit, 'upper')
                xml_upper.text = repr(axis_upper[axis1])
            
        # deal with this special case that needs a second axis
        if joint_subtype == 'universal':
            # figure out which axis is axis 2
            axis2 = -1
            for i in range(2,-1,-1):
                if axis_enabled[i]:
                    axis2 = i
                    break
            print("found axis2: ", axis2)
            
            # export data for that axis
            xml_axis2 = SubElement(xml_joint, 'axis2')
            xml_xyz2 = SubElement(xml_axis2, 'xyz')
            xml_xyz2.text = axis_dict[axis2]
            if axis_limit[axis2]:
                xml_limit2 = SubElement(xml_axis2, 'limit')
                xml_lower2 = SubElement(xml_limit2, 'lower')
                xml_lower2.text = repr(axis_lower[axis2])
                xml_upper2 = SubElement(xml_limit2, 'upper')
                xml_upper2.text = repr(axis_upper[axis2])

def export_options(xml_root): # TODO expose these in some sort of UI panel
    xml_options = SubElement(xml_root, "option")
    xml_options.set("timestep", "0.001")
    xml_options.set("iterations", "20")

    xml_sensorless = SubElement(xml_options, "flag")
    xml_sensorless.set("sensorless", "enable")

def export_setup_folders():
    if not os.path.exists(bpy.path.abspath('//mesh_stl')):
        print('Did not find mesh folder, creating it')
        os.makedirs(bpy.path.abspath('//mesh_stl'))

def export_pretty(context, xml_root):
    # make it user-readable and atually do the file export
    xml_pretty_string = xml_pretty(xml_root)
    print(xml_pretty_string)
    fd = open(bpy.path.abspath('//' + context.scene.robot_name + '.xml'), 'w')
    fd.write(xml_pretty_string)
    fd.close()

def export_defaults(): # TODO
    pass


# exports xml data for a link entity
# TODO handle multiple geometries for the same object  
def export_entity(context, obj, xml_model, body_is_root):

    if not body_is_root:
        # TODO position repr(obj.matrix_world.translation[0]) + " "
        # relative to the parent?
        pos = "0 0 0"

        # TODO orientation quaternion
        # relative to the parent?
        quat = "0 0 0 0"

    if obj.enum_sk_type == "body":
        print(f"Exporting {obj.name} as BODY")

        if body_is_root:
            # just worldbody with no info
            # this is fed in from up above
            xml_entity = xml_model
        else:
            xml_entity = SubElement(xml_model, 'body')
            xml_entity.set("name", obj.name)

            xml_entity.set("pos", pos)
            xml_entity.set("quat", quat)
            
            # TODO create mesh geoms if this object doesn't have its own geom child(s)

        for j, child in obj['sk_child_entity_list'].iteritems():
            print("calling child: ", child)
            export_entity(context, child, xml_entity, False)

    elif obj.enum_sk_type == "geom":
        print(f"Exporting {obj.name} as GEOM")
        xml_entity = SubElement(xml_model, "geom")
        xml_entity.set("name", obj.name)
        xml_entity.set("type", obj.sk_geom_type)
        xml_entity.set("size", calculate_geom_size_str(obj) )

        xml_entity.set("pos", pos)
        xml_entity.set("quat", quat)

    elif obj.enum_sk_type == "joint":
        print(f"Exporting {obj.name} as JOINT")
        # TODO how to handle multiple joints attached to the same body?
        # export_joint(context, joint, xml_entity, False)
        xml_entity = SubElement(xml_model, "joint")
        xml_entity.set("axis", obj.enum_joint_axis)
        xml_entity.set("type", obj.enum_joint_type)

        for j, child in obj['sk_child_entity_list'].iteritems():
            print("calling child: ", child)
            export_entity(context, child, xml_model, False)

    else:
        print(f"[ERROR] something has gone wrong exporting {obj.name}!")

    # recursively call function on child entities!
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

    xml_worldbody = SubElement(xml_root, "worldbody")
    
    # TODO search the scene for root object(s) - and static ground geometries
    # maybe have a world/ground empty? that can consistently start everything, have things referenced to it!

    # explore the kinematic tree from root down
    # TODO child bodies are XML within the parent body
    # TODO break up.. independence of geometry entries and bodies
    export_entity(context, root, xml_worldbody, True)

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
        size = [obj.scale[0], obj.scale[1], obj.scale[2]]

    if obj.sk_geom_type == "sphere":
        # radius
        size = [obj.scale[0]]

    if obj.sk_geom_type == "capsule":
        # radius, center length
        size = [obj.scale[1], 2*obj.scale[0]]

    if obj.sk_geom_type == "ellipsoid":
        # radius ; radius ; radius
        size = obj.scale

    if obj.sk_geom_type == "cylinder":
        # radius; length
        size = [obj.scale[1], 2*obj.scale[0]]

    if obj.sk_geom_type == "box":
        # x y z
        size = [2*obj.scale[0], 2*obj.scale[1],2*obj.scale[2]]    

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
            # TODO geometry primitives! 
            # use the empty cube type to help visualize what's going on? and look at its scale

            # TODO if body is to built of geoms, then those should drive the mass

            row = layout.row()
            row.prop(obj, "sk_parent_entity", text="Parent Entity")

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
            row.prop(obj, "sk_parent_entity", text="Parent Entity")

            row = layout.row()
            row.label(text="Joint Properties")
                   
            row = layout.row()
            row.prop(obj, 'enum_joint_type', text='joint type', expand=True)
            
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
    bpy.types.Object.enum_joint_axis = bpy.props.EnumProperty(items=enum_joint_axis_options, options = {"ENUM_FLAG"}, default={'x'})
    #bpy.types.Object.sk_child_entity = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_child_entity", description="Simple Kinematics Child Entity", update=None)

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

