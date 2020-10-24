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

def build_kinematic_tree(context):
    # tells links which joints are DOWNSTREAM of them
    # go over all in context.scene.objects
    for obj in context.scene.objects:
        obj['sk_link_child_joints'] = {}
    
    for obj in context.scene.objects:
        if not( obj.sk_joint_parent == None):
            dict_tmp = context.scene.objects[obj.sk_joint_parent.name]['sk_link_child_joints']
            dict_tmp[repr(len(dict_tmp.keys())+1)] = obj
            print(dict_tmp)    


def export_link(context, obj):
    print("exporting link", obj.name)
    # TODO export data for that link (special for root?????)

    
def export_joint(context, obj):
    print("exporting joint", obj.name)
    # TODO export data for that joint

    
def export_tree(context):
    root = context.object
    export_start_time = time.time()
    
    # explore the kinematic tree from root down
    link_frontier = [root]
    while len(link_frontier) > 0:
        link = link_frontier.pop()
        
        export_link(context, link)
        
        for j, joint in link['sk_link_child_joints'].iteritems():
            
            export_joint(context, joint)
            
            if joint.sk_joint_child.sk_export_time != export_start_time:
                # add to walk later
                link_frontier.append(joint.sk_joint_child)
                
                # mark link as visited so don't come back to it
                joint.sk_export_time = export_start_time
    

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
        # TODO additiona joint types
        

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
    bpy.types.Object.sk_export_time = bpy.props.FloatProperty(name='sk export time')
    bpy.utils.register_class(SDFExportOperator)
    bpy.utils.register_class(SimpleKinematicsJointPanel)


def unregister():
    bpy.utils.unregister_class(SDFExportOperator)
    bpy.utils.unregister_class(SimpleKinematicsJointPanel)
    del bpy.types.Object.enum_joint_type
    del bpy.types.Object.enum_joint_axis
    del bpy.types.Object.sk_joint_parent
    del bpy.types.Object.sk_joint_child

    del bpy.types.Object.sk_export_time


if __name__ == "__main__":
    register()


# TODO register shortcut key for adding the joint constraint? "j" seems free

