# Simple SDF Kinematics Exporter for Blender
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
    return reparsed.toprettyxml(indent="  ")

# explores all objects in context.scene.objects to 
# tells links which joints are DOWNSTREAM of them
def build_kinematic_tree(context):
    for obj in context.scene.objects:
        obj['sk_link_child_joints'] = {}
    
    for obj in context.scene.objects:
        if not( obj.sk_joint_parent == None):
            dict_tmp = context.scene.objects[obj.sk_joint_parent.name]['sk_link_child_joints']
            dict_tmp[repr(len(dict_tmp.keys())+1)] = obj

def get_joint_subtype(obj):
    if obj.enum_joint_type == 'revolute':
        dof = get_dof_qty(obj)
            
        if dof == 3:
            return "ball"
        if dof == 2:
            return "universal"
        
        if obj.sk_axis_x_limit or obj.sk_axis_y_limit or obj.sk_axis_z_limit:
            return "revolute"
        else:
            return "revolute" #TODO "continuous" is the right answer, but revolute is what DARTsim supports?
    else:
        return obj.enum_joint_type

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
    if obj.sk_link_collision == None:
        obj_collision = obj
    else:
        obj_collision = obj.sk_link_collision
        #obj_collision["sk_link_parent_joint"] = obj["sk_link_parent_joint"]

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


    

# http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification&
# exports xml data for a link entity
def export_link(context, obj, xml_model):
    print("exporting link", obj.name)
    xml_link = SubElement(xml_model, 'link')
    xml_link.set('name', obj.name)
    
    # pose is initial pose relative to the model frame
    # use blender origin as model frame origin
    xml_link_pose = SubElement(xml_link, 'pose')
    pose = ""
    pose += repr(obj.matrix_world.translation[0]) + " "
    pose += repr(obj.matrix_world.translation[1]) + " "
    pose += repr(obj.matrix_world.translation[2]) + " "
    
    rot = obj.matrix_world.to_euler('XYZ')
    pose += repr(rot.x) + " "
    pose += repr(rot.y) + " "
    pose += repr(rot.z)
    
    xml_link_pose.text = pose
    
    export_link_inertial(context, obj, xml_link)
    export_link_stl_visual(context, obj, xml_link)
    export_link_stl_collision(context, obj, xml_link)

# exports xml data for a joint entity
# warning there is a difference in joint definitions between SDF versions 1.4 and later    
def export_joint(context, obj, xml_model):
    print("exporting joint", obj.name)
    xml_joint = SubElement(xml_model, 'joint')
    xml_joint.set('name', obj.name)
    
    joint_subtype = get_joint_subtype(obj)
    xml_joint.set('type', joint_subtype)
    
    xml_joint_parent = SubElement(xml_joint, 'parent')
    xml_joint_parent.text = obj.sk_joint_parent.name
    xml_joint_child = SubElement(xml_joint, 'child')
    xml_joint_child.text = obj.sk_joint_child.name
    
    # TODO update to handle more types. seems like this could be a good use of OOP...
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
        
        # using SDF v1.5 standard
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

# follows an already-explored tree to add links and joints to the xml data    
def export_tree(context):
    root = context.object
    export_start_time = time.time()
    
    if not os.path.exists(bpy.path.abspath('//mesh_stl')):
        print('Did not find mesh folder, creating it')
        os.makedirs(bpy.path.abspath('//mesh_stl'))
    
    xml_root = Element('sdf')
    xml_root.set('version', '1.5')
    xml_model = SubElement(xml_root, 'model')
    xml_model.set('name', "robot_"+root.name)
    
    # explore the kinematic tree from root down
    link_frontier = [root]
    while len(link_frontier) > 0:
        link = link_frontier.pop()
        
        export_link(context, link, xml_model)
        
        for j, joint in link['sk_link_child_joints'].iteritems():
            
            export_joint(context, joint, xml_model)
            
            if (not hasattr(joint.sk_joint_child, 'sk_export_time')) or (joint.sk_joint_child['sk_export_time'] != export_start_time):
                # add to walk later
                link_frontier.append(joint.sk_joint_child)
                
                # mark link as visited so don't come back to it
                joint.sk_joint_child['sk_export_time'] = export_start_time
                
    xml_pretty_string = xml_pretty(xml_root)
    print(xml_pretty_string)
    fd = open(bpy.path.abspath('//rover.sdf'), 'w', newline='\n')
    #fd = open(bpy.path.abspath('//robot_' + root.name + '.sdf'), 'w', newline='\n')
    fd.write(xml_pretty_string)
    fd.close()
    
# this class is responsible for responding to the 'export' button being pushed
class SDFExportOperator(bpy.types.Operator):
    bl_idname = 'sk.export_sdf'
    bl_label = 'Export SDF'
    bl_description = 'Export SDF'
    bl_options = {'REGISTER', 'UNDO'}
    
    action: bpy.props.EnumProperty(
        items=[('SDF', 'sdf', 'sdf')]
        )
    
    def execute(self, context):
        print('Export action called, root object=', context.object)
        build_kinematic_tree(context)
        export_tree(context)
        print('export, successful')
        
        return {'FINISHED'}

def get_dof_qty(obj):
    dof = 0
    if obj.sk_axis_x_enabled == 1:
        dof += 1
    if obj.sk_axis_y_enabled == 1:
        dof += 1
    if obj.sk_axis_z_enabled == 1:
        dof += 1
    return dof



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
        row.prop(obj, 'enum_sk_type', text='Type', expand=True)
        
        if obj.enum_sk_type == 'link':
            row = layout.row()
            row.prop(obj, 'sk_mass', text='Mass (kg)')
                    
            row = layout.row()
            row.prop(obj, 'sk_link_collision', text='Collision Mesh (optional)')

            row = layout.row()
            row.operator('sk.export_sdf', text="Export using this object as root").action = 'SDF'

        if obj.enum_sk_type == 'joint':
            row = layout.row()
            row.label(text="Connects Links")
            
            row = layout.row()
            row.prop(obj, "sk_joint_parent")
            
            row = layout.row()
            row.prop(obj, "sk_joint_child")
        
            row = layout.row()
            row.label(text="Joint Properties")
                   
            row = layout.row()
            row.prop(obj, 'enum_joint_type', text='joint type', expand=True)
            
            if obj.enum_joint_type == 'revolute' or obj.enum_joint_type == 'prismatic':
                
                if obj.enum_joint_type == 'prismatic' and get_dof_qty(obj) > 1:
                    row = layout.row()
                    row.label(text="ERROR! Too many joint axis selected for this joint type")
                    
                if obj.enum_joint_type == 'revolute':
                    row = layout.row()
                    row.label(text="Joint subtype: " + get_joint_subtype(obj))
                    
                # a block for axis settings
                axis_layout = layout.row()
                
                # X axis
                axis_layout_x = axis_layout.column()
                row = axis_layout_x.row()
                axis_layout_x.prop(obj, 'sk_axis_x_enabled', text='X')
                
                row = axis_layout_x.row()
                row.prop(obj, 'sk_axis_x_limit', text='limit')
                row.enabled = obj.sk_axis_x_enabled
                
                col = axis_layout_x.column()
                if obj.enum_joint_type == 'prismatic':
                    col.prop(obj, 'sk_axis_x_lower_lin', text='lower limit')
                    col.prop(obj, 'sk_axis_x_upper_lin', text='upper limit')
                else:   
                    col.prop(obj, 'sk_axis_x_lower_rot', text='lower limit')
                    col.prop(obj, 'sk_axis_x_upper_rot', text='upper limit')
                col.enabled = obj.sk_axis_x_limit
                # TODO Prevent setting limits for ball joints
                
                # y axis
                axis_layout_y = axis_layout.column()
                row = axis_layout_y.row()
                row.prop(obj, 'sk_axis_y_enabled', text='Y')
                
                row = axis_layout_y.row()
                row.prop(obj, 'sk_axis_y_limit', text='limit')
                row.enabled = obj.sk_axis_y_enabled
                
                col = axis_layout_y.column()
                if obj.enum_joint_type == 'prismatic':
                    col.prop(obj, 'sk_axis_y_lower_lin', text='lower limit')
                    col.prop(obj, 'sk_axis_y_upper_lin', text='upper limit')
                else: 
                    col.prop(obj, 'sk_axis_y_lower_rot', text='lower limit')
                    col.prop(obj, 'sk_axis_y_upper_rot', text='upper limit')
                col.enabled = obj.sk_axis_y_limit
                
                # z axis
                axis_layout_z = axis_layout.column()
                row = axis_layout_z.row()
                row.prop(obj, 'sk_axis_z_enabled', text='Z')
                
                row = axis_layout_z.row()
                row.prop(obj, 'sk_axis_z_limit', text='limit')
                row.enabled = obj.sk_axis_z_enabled
                
                col = axis_layout_z.column()
                if obj.enum_joint_type == 'prismatic':
                    col.prop(obj, 'sk_axis_y_lower_lin', text='lower limit')
                    col.prop(obj, 'sk_axis_y_upper_lin', text='upper limit')
                else: 
                    col.prop(obj, 'sk_axis_z_lower_rot', text='lower limit')
                    col.prop(obj, 'sk_axis_z_upper_rot', text='upper limit')
                col.enabled = obj.sk_axis_z_limit
                

enum_joint_type_options = [
    ('revolute', 'Revolute', '', 1),
    ('prismatic', 'Prismatic', '', 2),
    ('fixed', 'Fixed', '', 3),
    ('screw', 'Screw', '', 4)
    ]
    
enum_joint_axis_options = [
    ('x', 'X', 'x axis'),
    ('y', 'Y', 'y axis'),
    ('z', 'Z', 'z axis')
    ]
    
enum_sk_type = [
    ('link', 'link', 'link'),
    ('joint', 'joint', 'joint')
    ]
    
def register():
    # step angle for UI drags
    step_angle_ui = 1500 # about 15 degrees
    step_lin_ui = 10 # 0.1m
    
    # create the needed properties
    bpy.types.Object.enum_sk_type = bpy.props.EnumProperty(items=enum_sk_type)
    bpy.types.Object.enum_joint_type = bpy.props.EnumProperty(items=enum_joint_type_options)
    bpy.types.Object.enum_joint_axis = bpy.props.EnumProperty(items=enum_joint_axis_options, options = {"ENUM_FLAG"}, default={'x'})
    bpy.types.Object.sk_joint_parent = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_joint_parent", description="Simple Kinematics Joint Parent Object", update=None)
    bpy.types.Object.sk_joint_child = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_joint_child", description="Simple Kinematics Joint Child Object", update=None)
    bpy.types.Object.sk_link_collision = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_link_collision", description="Simple Kinematics Link Collision Object", update=None)
    bpy.types.Object.sk_mass = bpy.props.FloatProperty(name='sk_mass', default=1)
    bpy.types.Object.sk_axis_x_enabled = bpy.props.BoolProperty(name="X", default=False)
    bpy.types.Object.sk_axis_x_limit = bpy.props.BoolProperty(name="x_limit", default=False)
    bpy.types.Object.sk_axis_x_lower_rot = bpy.props.FloatProperty(name="x_lower_rot", default=0, soft_min=-2*pi, soft_max=2*pi, unit="ROTATION", step=step_angle_ui)
    bpy.types.Object.sk_axis_x_upper_rot = bpy.props.FloatProperty(name="x_upper_rot", default=0, soft_min=-2*pi, soft_max=2*pi, unit="ROTATION", step=step_angle_ui)
    bpy.types.Object.sk_axis_x_lower_lin = bpy.props.FloatProperty(name="x_lower_lin", default=0, soft_min=-1, soft_max=1, unit="LENGTH", step=step_lin_ui)
    bpy.types.Object.sk_axis_x_upper_lin = bpy.props.FloatProperty(name="x_upper_lin", default=0, soft_min=-1, soft_max=1, unit="LENGTH", step=step_lin_ui)
    
    bpy.types.Object.sk_axis_y_enabled = bpy.props.BoolProperty(name="Y", default=False)
    bpy.types.Object.sk_axis_y_limit = bpy.props.BoolProperty(name="y_limit", default=False)
    bpy.types.Object.sk_axis_y_lower_rot = bpy.props.FloatProperty(name="y_lower_rot", default=0, soft_min=-2*pi, soft_max=2*pi, unit="ROTATION", step=step_angle_ui)
    bpy.types.Object.sk_axis_y_upper_rot = bpy.props.FloatProperty(name="y_upper_rot", default=0, soft_min=-2*pi, soft_max=2*pi, unit="ROTATION", step=step_angle_ui)
    bpy.types.Object.sk_axis_y_lower_lin = bpy.props.FloatProperty(name="y_lower_lin", default=0, soft_min=-1, soft_max=1, unit="LENGTH", step=step_lin_ui)
    bpy.types.Object.sk_axis_y_upper_lin = bpy.props.FloatProperty(name="y_upper_lin", default=0, soft_min=-1, soft_max=1, unit="LENGTH", step=step_lin_ui)
    
    bpy.types.Object.sk_axis_z_enabled = bpy.props.BoolProperty(name="Z", default=True)
    bpy.types.Object.sk_axis_z_limit = bpy.props.BoolProperty(name="z_limit", default=False)
    bpy.types.Object.sk_axis_z_lower_rot = bpy.props.FloatProperty(name="z_lower_rot", default=0, soft_min=-2*pi, soft_max=2*pi, unit="ROTATION", step=step_angle_ui)
    bpy.types.Object.sk_axis_z_upper_rot = bpy.props.FloatProperty(name="z_upper_rot", default=0, soft_min=-2*pi, soft_max=2*pi, unit="ROTATION", step=step_angle_ui)
    bpy.types.Object.sk_axis_z_lower_lin = bpy.props.FloatProperty(name="z_lower_lin", default=0, soft_min=-1, soft_max=1, unit="LENGTH", step=step_lin_ui)
    bpy.types.Object.sk_axis_z_upper_lin = bpy.props.FloatProperty(name="z_upper_lin", default=0, soft_min=-1, soft_max=1, unit="LENGTH", step=step_lin_ui)
    
    bpy.utils.register_class(SDFExportOperator)
    bpy.utils.register_class(SimpleKinematicsJointPanel)

def unregister():
    bpy.utils.unregister_class(SDFExportOperator)
    bpy.utils.unregister_class(SimpleKinematicsJointPanel)
    del bpy.types.Object.enum_joint_type
    del bpy.types.Object.enum_joint_axis
    del bpy.types.Object.sk_joint_parent
    del bpy.types.Object.sk_joint_child
    del bpy.types.Object.sk_link_collision
    del bpy.types.Object.enum_sk_type
    del bpy.types.Object.sk_mass

if __name__ == "__main__":
    register()
    print('register, successful')

