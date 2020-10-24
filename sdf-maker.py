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

import bpy


# this is the panel in the constraints window where you define joint information
class SimpleKinematicsJointPanel(bpy.types.Panel):
    """Creates a Panel in the Constraints properties window"""
    bl_label = "Simple Kinematics Joint Properties"
    bl_idname = "OBJECT_PT_simplekinematics"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "constraint"

    def draw(self, context):
        layout = self.layout

        obj = context.object

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
    bpy.utils.register_class(SimpleKinematicsJointPanel)


def unregister():
    bpy.utils.unregister_class(SimpleKinematicsJointPanel)
    del bpy.types.Object.enum_joint_type
    del bpy.types.Object.enum_joint_axis
    del bpy.types.Object.sk_joint_parent
    del bpy.types.Object.sk_joint_child


if __name__ == "__main__":
    register()





# define button to export the sdf model

# TODO how to walk through the model? need to declare a root body? how to detect things that have children based on the related constraints?
# generate a timecode/hash that gets saved for each export sessions
# how to handle loops so it stops? maybe on walking through the tree save that timecode property to each object
# so if you encounter the timecode, then don't parse that link
# for each link in links_frontier: # ok actually while not empty pop it off or something
#   export link data
#   for each child joint
#       save the joint export data
#       if the joint's child link hasn't been added to frontier yet
#          then add to frontier and mark down the export timecode so it's not visited again

# TODO register shortcut key for adding the joint constraint? "j" seems free

