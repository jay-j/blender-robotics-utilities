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

import bpy
import time
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

def build_kinematic_tree(context):
    # tells links which joints are DOWNSTREAM of them
    # go over all in context.scene.objects
    for obj in context.scene.objects:
        obj['sk_link_child_joints'] = {}
    
    for obj in context.scene.objects:
        if not( obj.sk_joint_parent == None):
            dict_tmp = context.scene.objects[obj.sk_joint_parent.name]['sk_link_child_joints']
            dict_tmp[repr(len(dict_tmp.keys())+1)] = obj

# http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification&
def export_link(context, obj, xml_model):
    print("exporting link", obj.name)
    # TODO export data for that link (special for root?????)
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


# warning there is a difference in joint definitions between SDF versions 1.4 and later    
def export_joint(context, obj, xml_model):
    print("exporting joint", obj.name)
    # TODO export data for that joint
    xml_joint = SubElement(xml_model, 'joint')
    xml_joint.set('name', obj.name)
    xml_joint.set('type', obj.enum_joint_type)
    xml_joint_parent = SubElement(xml_joint, 'parent')
    xml_joint_parent.text = obj.sk_joint_parent.name
    xml_joint_child = SubElement(xml_joint, 'child')
    xml_joint_child.text = obj.sk_joint_child.name
    
    # pose: where the joint is relative to the child frame, in the child frame
    # x y z angle angle angle (Euler roll pitch yaw; extrinsic x-y-z rotation)
    xml_joint_pose = SubElement(xml_joint, 'pose')
    
    
    # axis: <axis><xyz> unit vector..  <use_parent_model_frame>true</>
    # axis expressed in the parent link's model frame

    
def export_tree(context):
    root = context.object
    export_start_time = time.time()
    
    xml_root = Element('sdf')
    xml_root.set('version', '1.4')
    xml_model = SubElement(xml_root, 'model')
    xml_model.set('name', "robot_"+root.name)
    
    # explore the kinematic tree from root down
    link_frontier = [root]
    while len(link_frontier) > 0:
        link = link_frontier.pop()
        
        export_link(context, link, xml_model)
        
        for j, joint in link['sk_link_child_joints'].iteritems():
            
            export_joint(context, joint, xml_model)
            
            print("looking at joint", joint.name, "with child", joint.sk_joint_child.name)
            print("  time check", joint.sk_joint_child['sk_export_time'], export_start_time)
            
            if joint.sk_joint_child['sk_export_time'] != export_start_time:
                # add to walk later
                print("    adding", joint.sk_joint_child.name, "to search frontier")
                link_frontier.append(joint.sk_joint_child)
                
                # mark link as visited so don't come back to it
                joint.sk_joint_child['sk_export_time'] = export_start_time
                
                print("    time set", joint.sk_joint_child['sk_export_time'], export_start_time)
                
    print(xml_pretty(xml_root))
    

class SDFExportOperator(bpy.types.Operator):
    bl_idname = 'sk.export_sdf'
    bl_label = 'Export SDF'
    bl_description = 'Export SDF'
    bl_options = {'REGISTER', 'UNDO'}
    
    action: bpy.props.EnumProperty(
        items=[('SDF', 'sdf', 'sdf')]
        )
    
    def execute(self, context):
        print('Export action called!')
        root = context.object
        print("root object=", root)
        
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
        row.operator('sk.export_sdf', text="Export using this object as root").action = 'SDF'

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
        # TODO have separate panel menus for links and joints, add link mass
        

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

def register():
    # create the needed properties
    bpy.types.Object.enum_joint_type = bpy.props.EnumProperty(items=enum_joint_type_options)
    bpy.types.Object.enum_joint_axis = bpy.props.EnumProperty(items=enum_joint_axis_options, options = {"ENUM_FLAG"}, default={'x'})
    bpy.types.Object.sk_joint_parent = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_joint_parent", description="Simple Kinematics Joint Parent Object", update=None)
    bpy.types.Object.sk_joint_child = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_joint_child", description="Simple Kinematics Joint Child Object", update=None)
    #bpy.types.Object.sk_export_time = bpy.props.FloatProperty(name='sk export time')
    bpy.utils.register_class(SDFExportOperator)
    bpy.utils.register_class(SimpleKinematicsJointPanel)


def unregister():
    bpy.utils.unregister_class(SDFExportOperator)
    bpy.utils.unregister_class(SimpleKinematicsJointPanel)
    del bpy.types.Object.enum_joint_type
    del bpy.types.Object.enum_joint_axis
    del bpy.types.Object.sk_joint_parent
    del bpy.types.Object.sk_joint_child
    #del bpy.types.Object.sk_export_time


if __name__ == "__main__":
    register()


