# Simple URDF Kinematics Exporter for Blender
# Jay J.
# 2021


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

# spits out a directory structure with some boilerplate files so ros will take them

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

def create_folder(folder_name):
    working_dir = bpy.path.abspath('//')
    path = os.path.join(working_dir, folder_name)
    
    try:
        os.mkdir(path)
    except FileExistsError:
        pass

def boilerplate_config(robot_name):
    create_folder(robot_name + "/config")
    
    # create this joint name thing
    working_dir = bpy.path.abspath("//")
    path = os.path.join(working_dir, robot_name + "/config/joint_names_" + robot_name + ".yaml")
    fd = open(path, 'w')
    fd.write("controller_joint_names: ['', ]")
    fd.close()

def boilerplate_launch(robot_name):
    create_folder(robot_name + "/launch")
    
    # display.launch file
    xml_display = Element("launch")
    
    xml_display_1 = SubElement(xml_display, 'arg')
    xml_display_1.set("name", "model")
    
    xml_display_2 = SubElement(xml_display, 'arg')
    xml_display_2.set("name", "gui")
    xml_display_2.set("default", "False")
    
    xml_description = SubElement(xml_display, "param")
    xml_description.set("name", "robot_description")
    xml_description.set("textfile", "$(find " + robot_name + ")/urdf/" + robot_name + ".urdf")
    
    xml_usegui = SubElement(xml_display, "param")
    xml_usegui.set("name", "use_gui")
    xml_usegui.set("value", "$(arg gui)")
    
    xml_joint = SubElement(xml_display, "node")
    xml_joint.set("name", "joint_state_publisher")
    xml_joint.set("pkg", "joint_state_publisher")
    xml_joint.set("type", "joint_state_publisher")
    xml_joint_param = SubElement(xml_joint, "param")
    xml_joint_param.set("name","use_gui") # TODO in noetic need joint_state_publisher_gui package and do something different here
    xml_joint_param.set("value","True")    
    
    xml_state = SubElement(xml_display, "node")
    xml_state.set("name", "robot_state_publisher")
    xml_state.set("pkg", "robot_state_publisher")
    xml_state.set("type", "robot_state_publisher") # TODO future robot_state_publisher
    
    xml_rviz = SubElement(xml_display, "node")
    xml_rviz.set("name", "rviz")
    xml_rviz.set("pkg", "rviz")
    xml_rviz.set("type", "rviz")
    xml_rviz.set("args", "-d $(find " + robot_name + ")/urdf.rviz")
    
    xml_pretty_string = xml_pretty(xml_display)
    fd = open(bpy.path.abspath('//'+robot_name+'/launch/display.launch'), 'w')
    fd.write(xml_pretty_string)
    fd.close()

    # gazebo.launch file
    xml_gazebo = Element("launch")
    
    xml_include = SubElement(xml_gazebo, "include")
    xml_include.set("file", "$(find gazebo_ros)/launch/empty_world.launch")
    
    xml_tf = SubElement(xml_gazebo, "node")
    xml_tf.set("name", "tf_footprint_base")
    xml_tf.set("pkg", "tf")
    xml_tf.set("type", "static_transform_publisher")
    xml_tf.set("args", "0 0 0 0 0 0 base_link base_footprint 40") # TODO update to use the robot's base link?
    
    xml_spawn = SubElement(xml_gazebo, "node")
    xml_spawn.set("name", "spawn_model")
    xml_spawn.set("pkg", "gazebo_ros")
    xml_spawn.set("type", "spawn_model")
    xml_spawn.set("args", "-file $(find " + robot_name + ")/urdf/" + robot_name + ".urdf -urdf -model " + robot_name)
    xml_spawn.set("output", "screen")
    
    xml_topic = SubElement(xml_gazebo, "node")
    xml_topic.set("name", "fake_joint_calibration")
    xml_topic.set("pkg", "rostopic")
    xml_topic.set("type", "rostopic")
    xml_topic.set("args", "pub /calibrated std_msgs/Bool true")
    
    xml_pretty_string = xml_pretty(xml_gazebo)
    fd = open(bpy.path.abspath('//'+robot_name+'/launch/gazebo.launch'), 'w')
    fd.write(xml_pretty_string)
    fd.close()

def boilerplate_cmake(robot_name):
    fd = open(bpy.path.abspath("//"+robot_name+"/CMakeLists.txt"), "w")
    fd.write("cmake_minimum_required(VERSION 2.8.3)\nproject(" + robot_name +")\nfind_package(catkin REQUIRED)\ncatkin_package()\nfind_package(roslaunch)\nforeach(dir config launch meshes urdf)\n\tinstall(DIRECTORY ${dir}/\n\t\tDESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})\nendforeach(dir)")
    fd.close()

def boilerplate_package(robot_name):
    fd = open(bpy.path.abspath("//"+robot_name+"/package.xml"), "w")
    fd.write("<package format=\"2\">\n<name>"+robot_name+"</name>\n<version>1.0.0</version>\n<description>\n<p>URDF Description package for "+robot_name+"</p>\n<p>This package contains configuration data, 3D models and launch files for " + robot_name + " robot</p>\n</description>\n <author>TODO</author>\n<maintainer email=\"TODO@email.com\" />\n<license>BSD</license>\n<buildtool_depend>catkin</buildtool_depend>\n<depend>roslaunch</depend>\n<depend>robot_state_publisher</depend>\n<depend>rviz</depend>\n<depend>joint_state_publisher</depend>\n<depend>gazebo</depend>\n<export>\n<architecture_independent />\n</export>\n</package>")
    fd.close()

def boilerplate(robot_name):
    create_folder(robot_name)
    create_folder(robot_name + "/meshes")
    create_folder(robot_name + "/textures")
    create_folder(robot_name + "/urdf")
    
    boilerplate_config(robot_name)
    boilerplate_launch(robot_name)
    boilerplate_cmake(robot_name)
    boilerplate_package(robot_name)
   
    print("done creating boilerplate")

####################################################################################################################################
# explores all objects in context.scene.objects to 
# tells links which joints are DOWNSTREAM of them
meshes_exported = []
def build_kinematic_tree(context):
    meshes_exported.clear()
    for obj in context.scene.objects:
        obj['sk_link_child_joints'] = {}
        obj['sk_link_parent_joint'] = None
    
    for obj in context.scene.objects:
        if not( obj.sk_joint_parent == None):
            dict_tmp = context.scene.objects[obj.sk_joint_parent.name]['sk_link_child_joints']
            dict_tmp[repr(len(dict_tmp.keys())+1)] = obj

def get_joint_subtype(obj):
    if obj.enum_joint_type == 'revolute':
        dof = get_dof_qty(obj)
            
        if dof == 3:
            return "ball (not supported in URDF!)"
        if dof == 2:
            return "universal (not supported in URDF!)"
        
        if obj.sk_axis_x_limit or obj.sk_axis_y_limit or obj.sk_axis_z_limit:
            return "revolute"
        else:
            return "continuous"
    elif obj.enum_joint_type == 'prismatic':
        dof = get_dof_qty(obj)
        
        if dof == 1:
            return 'prismatic'
        if dof == 2:
            return 'planar'
        if dof == 3:
            return 'not supported in URDF!'
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

def xml_origin_wrt_parent(context, obj, xml_entity):
     # get pose relative to parent joint
    if obj['sk_link_parent_joint'] == None:
        pose_wrt_parent = obj.matrix_world
    else:
        if context.scene.local_meshes:
           pose_wrt_parent = obj['sk_link_parent_joint'].matrix_world.inverted() @ obj.matrix_world
        else:
          pose_wrt_parent = obj['sk_link_parent_joint'].matrix_world.inverted() # different because we don't control STL origin... TODO
    
    pose_xyz = ''
    pose_xyz += repr(pose_wrt_parent.translation[0]) + " "
    pose_xyz += repr(pose_wrt_parent.translation[1]) + " "
    pose_xyz += repr(pose_wrt_parent.translation[2]) + " "
    rot = pose_wrt_parent.to_euler('XYZ')
    pose_rpy = ''
    pose_rpy += repr(rot.x) + " "
    pose_rpy += repr(rot.y) + " "
    pose_rpy += repr(rot.z)

    xml_inertial_origin = SubElement(xml_entity, 'origin')
    xml_inertial_origin.set('xyz', pose_xyz)
    xml_inertial_origin.set('rpy', pose_rpy)
    

def export_link_inertial(context, obj, xml_link):
    xml_inertial = SubElement(xml_link, 'inertial')
    xml_origin_wrt_parent(context, obj, xml_inertial)
    
    xml_mass = SubElement(xml_inertial, 'mass')
    xml_mass.set('value', repr(obj.sk_mass))
    
    xml_inertia = SubElement(xml_inertial, 'inertia')
    inertia = inertia_of_box(obj)
    for i in inertia:
        xml_inertia.set(i, repr(inertia[i]))

def export_stl(context, obj, xml_geometry):
    mesh_data_name = obj.data.name

    # if not using local_meshes option, then have to export a new object for sure
    # if yes using local_meshes option, then only export if the mesh hasn't been exported yet
    if not context.scene.local_meshes or meshes_exported.count(mesh_data_name) == 0:

        # store the active object so it can be restored later
        sel_obj = bpy.context.view_layer.objects.active
        
        # TODO find a way for collection instances to serve as single links
        if obj.type != 'MESH':
            print("ERROR object", obj.name, "is not a mesh object")
            return
        
        # select only the single mesh to be exported
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        
        if context.scene.local_meshes:
            # CAUTION! the option use_global_frame requres modifying Blender D11517 (https://developer.blender.org/D11517)
            bpy.ops.export_mesh.stl(filepath=bpy.path.abspath('//'+context.scene.robot_name+'/meshes/' + mesh_data_name + '.stl'), use_selection=True, use_global_frame=False)
        else:
            bpy.ops.export_mesh.stl(filepath=bpy.path.abspath('//'+context.scene.robot_name+'/meshes/' + obj.name + '.stl'), use_selection=True) # after D11517, this has a default argument use_global_frame=True so this line will still work
        
        # restore selection
        bpy.ops.object.select_all(action='DESELECT')
        sel_obj.select_set(True)
    
    xml_mesh = SubElement(xml_geometry, 'mesh')
    if context.scene.local_meshes:
        xml_mesh.set('filename', 'package://'+ context.scene.robot_name + "/meshes/" + mesh_data_name + '.stl')
    else:
        xml_mesh.set('filename', 'package://'+ context.scene.robot_name + "/meshes/" + obj.name + '.stl')
    

def export_link_collision_stl(context, obj, xml_link):
    xml_collision = SubElement(xml_link, 'collision')
    
    # change to collision object if one is defined
    if obj.sk_link_collision == None:
        obj_collision = obj
    else:
        obj_collision = obj.sk_link_collision
        obj_collision["sk_link_parent_joint"] = obj["sk_link_parent_joint"]
    
    xml_origin_wrt_parent(context, obj_collision, xml_collision)

    xml_geometry = SubElement(xml_collision, 'geometry')
    export_stl(context, obj_collision, xml_geometry)

def export_link_visual_stl(context, obj, xml_link):
    # get the pose
    xml_visual = SubElement(xml_link, 'visual')

    if obj['sk_link_parent_joint'] == None:
            pose_wrt_parent = obj.matrix_world
    else:
        if context.scene.local_meshes:
            pose_wrt_parent = obj['sk_link_parent_joint'].matrix_world.inverted() @ obj.matrix_world
        else:
            pose_wrt_parent = obj['sk_link_parent_joint'].matrix_world.inverted() # different because we don't control STL origin... TODO
    
    pose_xyz = ''
    pose_xyz += repr(pose_wrt_parent.translation[0]) + " "
    pose_xyz += repr(pose_wrt_parent.translation[1]) + " "
    pose_xyz += repr(pose_wrt_parent.translation[2]) + " "
    rot = pose_wrt_parent.to_euler('XYZ')
    pose_rpy = ''
    pose_rpy += repr(rot.x) + " "
    pose_rpy += repr(rot.y) + " "
    pose_rpy += repr(rot.z)

    xml_inertial_origin = SubElement(xml_visual, 'origin')
    xml_inertial_origin.set('xyz', pose_xyz)
    xml_inertial_origin.set('rpy', pose_rpy)

    xml_geometry = SubElement(xml_visual, 'geometry')
    export_stl(context, obj, xml_geometry)
    
# https://wiki.ros.org/urdf/XML
# http://wiki.ros.org/urdf/Tutorials
# exports xml data for a link entity
def export_link(context, obj, xml_model):
    print("exporting link", obj.name)
    # print("parent joint is:", obj['sk_link_parent_joint'])
    
    xml_link = SubElement(xml_model, 'link')
    xml_link.set('name', obj.name)
        
    export_link_inertial(context, obj, xml_link)
    export_link_visual_stl(context, obj, xml_link)
    export_link_collision_stl(context, obj, xml_link)

# exports xml data for a joint entity
def export_joint(context, obj, xml_model):
    print("exporting joint", obj.name)
    xml_joint = SubElement(xml_model, 'joint')
    xml_joint.set('name', obj.name)
    
    joint_subtype = get_joint_subtype(obj)
    xml_joint.set('type', joint_subtype)
    
    xml_joint_parent = SubElement(xml_joint, 'parent')
    xml_joint_parent.set('link', obj.sk_joint_parent.name)
    xml_joint_child = SubElement(xml_joint, 'child')
    xml_joint_child.set('link', obj.sk_joint_child.name)
    
    # pose: where the joint is relative to the parent frame, in the parent JOINT frame
    # x y z angle angle angle (Euler roll pitch yaw; extrinsic x-y-z rotation)
    xml_joint_origin = SubElement(xml_joint, 'origin')

    print("  joint matrix:", obj.matrix_world)
    print("  joint parent:", obj.sk_joint_parent)
    print("  joint parent matrix:", obj.sk_joint_parent.matrix_world)

    if obj.sk_joint_parent['sk_link_parent_joint'] == None:
        pose_wrt_parent = obj.matrix_world
    else:
        pose_wrt_parent = obj.sk_joint_parent['sk_link_parent_joint'].matrix_world.inverted() @ obj.matrix_world
        
    print("  transform wrt parent:\n", pose_wrt_parent)

    origin_xyz = ""
    origin_xyz += repr(pose_wrt_parent.translation[0]) + " "
    origin_xyz += repr(pose_wrt_parent.translation[1]) + " "
    origin_xyz += repr(pose_wrt_parent.translation[2]) + " "
    
    rot = pose_wrt_parent.to_euler('XYZ')
    origin_rpy = ""
    origin_rpy += repr(rot.x) + " "
    origin_rpy += repr(rot.y) + " "
    origin_rpy += repr(rot.z)
    
    xml_joint_origin.set('xyz', origin_xyz)
    xml_joint_origin.set('rpy', origin_rpy)
        
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
        xml_axis.set('xyz', axis_dict[axis1])
        
        # set axis limits for those that allow it
        if ['revolute', 'prismatic'].count(joint_subtype) > 0:
            if axis_limit[axis1]:
                xml_axis_limit = SubElement(xml_joint, 'limit')
                xml_axis_limit.set('lower', repr(axis_lower[axis1]))
                xml_axis_limit.set('upper', repr(axis_upper[axis1]))
                xml_axis_limit.set('effort', repr(1e3)) # TODO
                xml_axis_limit.set('velocity', repr(1e3)) # TODO
            
        # deal with this special case that needs a second axis # TODO broken for URDFs?
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
            xml_axis2.set('xyz', axis_dict[axis2])
            if axis_limit[axis2]:
                xml_axis_limit2 = SubElement(xml_axis2, 'limit')
                xml_axis_limit2.set('lower', repr(axis_lower[axis2]))
                xml_axis_limit2.set('upper', repr(axis_upper[axis2]))
                xml_axis_limit2.set('effort', repr(1e3)) # TODO
                xml_axis_limit2.set('velocity', repr(1e3)) # TODO

# follows an already-explored tree to add links and joints to the xml data    
def export_tree(context):
    root = context.object
    export_start_time = time.time()
    
    xml_root = Element('robot')
    xml_root.set('name', context.scene.robot_name)
    
    # explore the kinematic tree from root down
    link_frontier = [root]
    while len(link_frontier) > 0:
        link = link_frontier.pop()
        
        export_link(context, link, xml_root)
        
        for j, joint in link['sk_link_child_joints'].iteritems():
            
            export_joint(context, joint, xml_root)
            
            if (not hasattr(joint.sk_joint_child, 'sk_export_time')) or (joint.sk_joint_child['sk_export_time'] != export_start_time):
                # add to walk later
                link_frontier.append(joint.sk_joint_child)
                
                # mark link as visited so don't come back to it
                joint.sk_joint_child['sk_export_time'] = export_start_time
                joint.sk_joint_child['sk_link_parent_joint'] = joint
                
    xml_pretty_string = xml_pretty(xml_root)
    fd = open(bpy.path.abspath("//"+context.scene.robot_name+"/urdf/" + context.scene.robot_name + ".urdf"), "w")
    fd.write(xml_pretty_string)
    fd.close()
    
# this class is responsible for responding to the 'export' button being pushed
class URDFExportOperator(bpy.types.Operator):
    bl_idname = 'sk.export_urdf'
    bl_label = 'Export URDF'
    bl_description = 'Export URDF'
    bl_options = {'REGISTER', 'UNDO'}
    
    action: bpy.props.EnumProperty(
        items=[('URDF', 'urdf', 'urdf')]
        )
    
    def execute(self, context):
        print('Export action called, root object=', context.object)
        boilerplate(context.scene.robot_name)
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
        row.prop(context.scene, "robot_name", text="Robot Name")

        row = layout.row()
        row.prop(context.scene, "local_meshes", text="Use local meshes (shared when possible)")
        
        row = layout.row()
        row.prop(obj, 'enum_sk_type', text='Type', expand=True)
        
        if obj.enum_sk_type == 'link':
            row = layout.row()
            row.prop(obj, 'sk_mass', text='Mass (kg)')
            
            row = layout.row()
            row.prop(obj, 'sk_link_collision', text='Collision Mesh (optional)')
                    
            row = layout.row()
            row.operator('sk.export_urdf', text="Export using this object as root").action = 'URDF'

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
                
                if obj.enum_joint_type == 'prismatic' and get_dof_qty(obj) > 2:
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
    ('Floating', 'Floating', '', 5)
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
    
    bpy.types.Scene.robot_name = bpy.props.StringProperty(name="robot_name", default="")
    bpy.types.Scene.local_meshes = bpy.props.BoolProperty(name="local_meshes", default=False, description="Local coordinates and shared mesh files if you have https://developer.blender.org/D11517")
    
    bpy.utils.register_class(URDFExportOperator)
    bpy.utils.register_class(SimpleKinematicsJointPanel)

def unregister():
    bpy.utils.unregister_class(URDFExportOperator)
    bpy.utils.unregister_class(SimpleKinematicsJointPanel)
    del bpy.types.Object.enum_joint_type
    del bpy.types.Object.enum_joint_axis
    del bpy.types.Object.sk_joint_parent
    del bpy.types.Object.sk_joint_child
    del bpy.types.Object.sk_link_collision
    del bpy.types.Object.enum_sk_type
    del bpy.types.Object.sk_mass
    del bpy.types.Scene.robot_name
    del bpy.types.Scene.local_meshes

if __name__ == "__main__":
    register()
    print('register, successful')

