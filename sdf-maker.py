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


class SimpleKinematicsPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
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
    
        # TODO need to detect property changes to the parent/child so they can be marked/unmarked rather than searching at runtime for them
        
        row = layout.row()
        row.label(text="Joint Properties")
        
        row = layout.row()
        row.label(text="Type: Revolute")
        
        
        
        
        # TODO put in axis with limits
        
        
def joint_parent_changed(self, context):
    print("joint parent property changed!")
    print("current object is", context.object)
    print(self.sk_joint_parent)
    print(context.object.sk_joint_parent)

def register():
    bpy.utils.register_class(SimpleKinematicsPanel)
    
    # create the properties I need to 
    # bpy.props
    bpy.types.Object.sk_joint_parent = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_joint_parent", description="Simple Kinematics Joint Parent Object", update=joint_parent_changed)
    bpy.types.Object.sk_joint_child = bpy.props.PointerProperty(type=bpy.types.Object, name="sk_joint_child", description="Simple Kinematics Joint Child Object", update=None)


def unregister():
    bpy.utils.unregister_class(SimpleKinematicsPanel)


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

