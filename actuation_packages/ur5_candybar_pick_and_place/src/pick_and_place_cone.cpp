#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

void addObjectToPlanningScene(moveit::planning_interface::MoveGroupInterface& move_group_interface_arm) 
{

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    // Add the object to be grasped (the square box) to the planning scene
    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();

    collision_object.id = "pipe";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.03;

    geometry_msgs::Pose cone_pose;
    cone_pose.orientation.w = 0.0;
    cone_pose.position.x = 0.0;
    cone_pose.position.y = 0.8;
    cone_pose.position.z = 0.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cone_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_objects);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");

    ros::Duration(0.1).sleep();

    // Allow collisions between the gripper and the box to be able to grasp it
    planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix& acm = ls->getAllowedCollisionMatrixNonConst();
    acm.setEntry("pipe", "robotiq_85_left_finger_tip_link", true);
    acm.setEntry("pipe", "robotiq_85_right_finger_tip_link", true);
    std::cout << "\nAllowedCollisionMatrix:\n";
    acm.print(std::cout);
    moveit_msgs::PlanningScene diff_scene;
    ls->getPlanningSceneDiffMsg(diff_scene);

    planning_scene_interface.applyPlanningScene(diff_scene); 

    ros::Duration(0.1).sleep();
}

void moveToReadyPose(moveit::planning_interface::MoveGroupInterface::Plan& my_plan_arm, moveit::planning_interface::MoveGroupInterface& move_group_interface_arm) 
{
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));

    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Planning Home position: %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    //2. Orient the TCP so that its horizontal
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");

    geometry_msgs::Pose target_pose1;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(M_PI,0,M_PI/2);
    quat_tf = quat_tf.normalize();
    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(quat_tf);
    target_pose1.orientation = quat_msg;
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.5;
    target_pose1.position.z = 0.2;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Planning TCP horizontal position: %s", success ? "" : "FAILED");

    move_group_interface_arm.move();
}

void openGripper(moveit::planning_interface::MoveGroupInterface::Plan& my_plan_gripper, moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper) 
{
    //3. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
    bool success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Planning to open gripper %s", success ? "" : "FAILED");
    move_group_interface_gripper.move();
}

void closeGripper(moveit::planning_interface::MoveGroupInterface::Plan& my_plan_gripper, moveit::planning_interface::MoveGroupInterface& move_group_interface_gripper) 
{
    // 5. Close the  gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));

    bool success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Planning gripper close %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle n;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);


    //Adding candy bar to planning scene
    // addObjectToPlanningScene(move_group_interface_arm);

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    //initialize plan objects
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;


    //move to ready position
    moveToReadyPose(my_plan_arm, move_group_interface_arm);

    //open gripper
    openGripper(my_plan_gripper, move_group_interface_gripper);



    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");
    geometry_msgs::Pose target_pose1;

    // 4. Move the TCP close to the object
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.67;
    target_pose1.position.z = 0.1;
    move_group_interface_arm.setPoseTarget(target_pose1);

    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Planning to Move TCP close to object %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    closeGripper(my_plan_gripper, move_group_interface_gripper);

    // 6. Lift the Candy Bar up
    target_pose1.position.z = target_pose1.position.z + 0.2;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Planning to Lift Candy Bar up %s", success ? "" : "FAILED");

    move_group_interface_arm.move();


    // //7. Move Candy bar to the spigot hole
    // // target_pose1.position.x = target_pose1.position.x + 1.6;
    // target_pose1.position.y = 0.2;
    

    // move_group_interface_arm.setPoseTarget(target_pose1);

    // moveit::core::MoveItErrorCode res = move_group_interface_arm.plan(my_plan_arm);
    // success = (res == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // std::cout << "fail error code: " << res << "\n";
    // ROS_INFO_NAMED("tutorial", "Planning to Move Candy Bar to Spigot Hole PART 2 %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();


    // target_pose1.position.x = 1.6;
    // target_pose1.position.y = 0;
    // target_pose1.position.z = 0.1;
    

    // move_group_interface_arm.setPoseTarget(target_pose1);

    // res = move_group_interface_arm.plan(my_plan_arm);
    // success = (res == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // std::cout << "fail error code: " << res << "\n";
    // ROS_INFO_NAMED("tutorial", "Planning to Move Candy Bar to Spigot Hole %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();



    // // 8. Lower Candy Bar
    // target_pose1.position.z = target_pose1.position.z - 0.4;

    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Planning to Lower Candy Bar %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // // 9. Open the gripper

    // move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));

    // success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Planning to open gripper %s", success ? "" : "FAILED");

    // move_group_interface_gripper.move();

    // //10. Exit 
    // target_pose1.position.y = target_pose1.position.y - 0.2;

    // move_group_interface_arm.setPoseTarget(target_pose1);

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Planning to Lower Candy Bar %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    // //11. Go back to home pose
    // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));

    // success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("tutorial", "Planning Home position: %s", success ? "" : "FAILED");

    // move_group_interface_arm.move();

    ros::shutdown();
    return 0;
}
