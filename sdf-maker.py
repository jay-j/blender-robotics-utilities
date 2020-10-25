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

def export_link_stl(context, obj, xml_link):
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
    
    # get the pose
    xml_visual = SubElement(xml_link, 'visual')
    xml_visual.set('name', obj.name + '_visual')
    xml_geometry = SubElement(xml_visual, 'geometry')
    
    xform = obj.matrix_world.inverted()
    pose = ''
    pose += repr(xform.translation[0]) + " "
    pose += repr(xform.translation[1]) + " "
    pose += repr(xform.translation[2]) + " "
    rot = xform.to_euler('XYZ')
    pose += repr(rot.x) + " "
    pose += repr(rot.y) + " "
    pose += repr(rot.z)
    
    xml_mesh = SubElement(xml_geometry, 'mesh')
    xml_mesh_uri = SubElement(xml_mesh, 'uri')
    xml_mesh_uri.text = 'file://'+ bpy.path.abspath('//mesh_stl/' + obj.name + '.stl')

    xml_geo_pose = SubElement(xml_visual, 'pose')
    xml_geo_pose.text = pose
    

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
    export_link_stl(context, obj, xml_link)

# exports xml data for a joint entity
# warning there is a difference in joint definitions between SDF versions 1.4 and later    
def export_joint(context, obj, xml_model):
    print("exporting joint", obj.name)
    xml_joint = SubElement(xml_model, 'joint')
    xml_joint.set('name', obj.name)
    xml_joint.set('type', obj.enum_joint_type)
    xml_joint_parent = SubElement(xml_joint, 'parent')
    xml_joint_parent.text = obj.sk_joint_parent.name
    xml_joint_child = SubElement(xml_joint, 'child')
    xml_joint_child.text = obj.sk_joint_child.name
    
    # TODO update to handle more types. seems like this could be a good use of OOP...
    if obj.enum_joint_type == 'revolute':
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
        
    if obj.enum_joint_type == 'revolute':
        # using SDF v1.5 standard
        # axis unit vector expressed in the joint frame
        axis_dict = {'x':'1 0 0', 'y':'0 1 0', 'z':'0 0 1'}
        xml_axis = SubElement(xml_joint, 'axis')
        xml_xyz = SubElement(xml_axis, 'xyz')
        xml_xyz.text = axis_dict[list(obj.enum_joint_axis)[0]]

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
            
            if joint.sk_joint_child['sk_export_time'] != export_start_time:
                # add to walk later
                link_frontier.append(joint.sk_joint_child)
                
                # mark link as visited so don't come back to it
                joint.sk_joint_child['sk_export_time'] = export_start_time
                
    xml_pretty_string = xml_pretty(xml_root)
    print(xml_pretty_string)
    fd = open(bpy.path.abspath('//robot_' + root.name + '.sdf'), 'w')
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
        
        return {'FINISHED'}

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
            row.operator('sk.export_sdf', text="Export using this object as root").action = 'SDF'

        if obj.enum_sk_type == 'joint':
            row = layout.row()
            row.label(text="Connects Links")
            
            row = layout.row()
            row.prop(obj, "sk_joint_parent")
            
            row = layout.row()
            row.prop(obj, "sk_joint_child")
        
            # TODO better to detect property changes to the parent/child so they can be marked/unmarked rather than searching at export time for them
            
            row = layout.row()
            row.label(text="Joint Properties")
                   
            row = layout.row()
            row.prop(obj, 'enum_joint_type', text='joint type', expand=True)
            
            if obj.enum_joint_type == 'revolute' or obj.enum_joint_type == 'prismatic':
                row = layout.row()
                row.label(text="Joint Axis:")
                row.prop(obj, 'enum_joint_axis', text='joint axis', expand=True)
                
                if len(obj.enum_joint_axis) > 1:
                    row = layout.row()
                    row.label(text="ERROR! Too many joint axis selected for this joint type")

            # TODO add limits to joint angles
            # TODO additional joint types
        

enum_joint_type_options = [
    ('revolute', 'Revolute', '', 1),
    ('prismatic', 'Prismatic', '', 2),
    ('todo', 'todo more', '', 3),
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
    
enum_sk_vis_type = [
    ('box','box','box'),
    ('mesh','mesh','mesh')
    ]

def register():
    # create the needed properties
    bpy.types.Object.enum_sk_type = bpy.props.EnumProperty(items=enum_sk_type)
    bpy.types.Object.enum_sk_vis_type = bpy.props.EnumProperty(items=enum_sk_vis_type)
    bpy.types.Object.enum_joint_type = bpy.props.EnumProperty(items=enum_joint_type_options)
    bpy.types.Object.enum_joint_axis = bpy.props.EnumProperty(items=enum_joint_axis_options, options = {"ENUM_FLAG"}, default={'x'})
    bpy.types.Object.sk_joint_parent = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_joint_parent", description="Simple Kinematics Joint Parent Object", update=None)
    bpy.types.Object.sk_joint_child = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_joint_child", description="Simple Kinematics Joint Child Object", update=None)
    bpy.types.Object.sk_mass = bpy.props.FloatProperty(name='sk_mass', default=1)
    bpy.utils.register_class(SDFExportOperator)
    bpy.utils.register_class(SimpleKinematicsJointPanel)

def unregister():
    bpy.utils.unregister_class(SDFExportOperator)
    bpy.utils.unregister_class(SimpleKinematicsJointPanel)
    del bpy.types.Object.enum_joint_type
    del bpy.types.Object.enum_joint_axis
    del bpy.types.Object.sk_joint_parent
    del bpy.types.Object.sk_joint_child
    del bpy.types.Object.enum_sk_type
    del bpy.types.Object.sk_mass

if __name__ == "__main__":
    register()


