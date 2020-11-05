#include <iostream>
#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/gui.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;

class MyWindow : public dart::gui::glut::SimWindow{
public:
    MyWindow(WorldPtr world){
        setWorld(world);
    }


    void timeStepping() override {
        SimWindow::timeStepping();
    }
};


SkeletonPtr create_base_skeleton(){
    SkeletonPtr skeleton = Skeleton::create("skeleton");
    WeldJoint::Properties properties = WeldJoint::Properties();
    properties.mName = "root_weld_joint";
    properties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0, 0.0, 0);
    BodyNodePtr root = skeleton->createJointAndBodyNodePair<WeldJoint>(nullptr, properties, BodyNode::AspectProperties("root")).second;
    return skeleton;
}

int main(int argc, char* argv[]){
    // create a world
    dart::simulation::WorldPtr world(new dart::simulation::World);
    
    // create an empty skeleton
    SkeletonPtr skeleton = create_base_skeleton();
    
    // load the sdf file, add to the world
    auto robot = dart::utils::SdfParser::readSkeleton("/home/jjasper/blender-sdf-quick/robot.sdf");
    robot->disableSelfCollisionCheck();
    
    
    // now weld the new structure to the empty one
    WeldJoint::Properties properties = WeldJoint::Properties();
    properties.mName = "root2_weld_joint";
    properties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0, 0.0, 0);
    
    robot->getRootBodyNode()->moveTo<WeldJoint>(skeleton->getRootBodyNode(), properties);
    
    world->addSkeleton(skeleton);

    
    // set world gravity
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    
    // create a window for showing stuff!
    MyWindow window(world);
    
    glutInit(&argc, argv);
    window.initWindow(640*2, 480*2, "sdf display");
    glutMainLoop();
    
    return 0;
}
