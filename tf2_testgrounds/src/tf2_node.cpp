#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
//#include <tf2_testgrounds/SendTarget.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::Point objectLocationGlobal;
geometry_msgs::Pose robotLocationOdom;

geometry_msgs::Transform map2odom;  // Constant as both are fixed
geometry_msgs::Transform odom2base;

tf2_ros::Buffer tf_buffer;

geometry_msgs::Point transformFrames(geometry_msgs::Point p, const std::string from, const std::string to)
{
  geometry_msgs::PoseStamped inputPose;
  inputPose.pose.position = p;
  inputPose.header.frame_id = from;
  inputPose.header.stamp = ros::Time(0);

  geometry_msgs::PoseStamped outputPose = tf_buffer.transform(inputPose, to, ros::Duration(1.0));

  return outputPose.pose.position;
}

// bool send_target(tf2_testgrounds::SendTarget::Request &req,
//                  tf2_testgrounds::SendTarget::Response &res)
// {
//   if (req.a == 13)
//   {
//     ROS_INFO("Request received");

//     geometry_msgs::Pose targetPose; // Assuming relative to robot base
    
//     // calculate targetpose using frame transformations

//     targetPose.position = transformFrames(objectLocationGlobal, "map", "robot");

//     // Other calculations

//     res.targetPose = targetPose;

//     ROS_INFO("Target pose sent out");
//   }

//   return true;
// }

geometry_msgs::PoseStamped getTargetPosition()
{
  geometry_msgs::PoseStamped targetPose;

  targetPose.pose.position = transformFrames(objectLocationGlobal, "map", "robot");
  // Other calculations to get target pose

  targetPose.header.frame_id = "robot";
  targetPose.header.stamp = ros::Time::now();

  return targetPose;
}

void locationCallback(nav_msgs::Odometry msg)
{
  ROS_INFO_STREAM("Received location data");

  // Separate by header if map or odom - only take odom
  if (msg.header.frame_id == "odom")
  {
    robotLocationOdom = msg.pose.pose;
  }
}

void objectCallback(geometry_msgs::Point msg)
{
  ROS_INFO_STREAM("Received candybar location data");

  // Assume location is in map frame
  objectLocationGlobal = msg;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_node");

  ros::NodeHandle node;

  // Initialise tf2 nodes
  tf2_ros::StaticTransformBroadcaster br;
  tf2_ros::TransformListener listener(tf_buffer);

  // Define static frame transformations
  geometry_msgs::TransformStamped base2robot;
  geometry_msgs::TransformStamped base2camera;

  base2robot.header.stamp = ros::Time::now();
  base2robot.header.frame_id = "base_link";
  base2robot.child_frame_id = "robot";
  base2robot.transform.translation.x = 0.0;
  base2robot.transform.translation.y = 0.0;
  base2robot.transform.translation.z = 0.0;
  base2robot.transform.rotation.x = 0.0;
  base2robot.transform.rotation.y = 0.0;
  base2robot.transform.rotation.z = 0.0;
  base2robot.transform.rotation.w = 1.0;

  base2camera.header.stamp = ros::Time::now();
  base2camera.header.frame_id = "base_link";
  base2camera.child_frame_id = "camera";
  base2camera.transform.translation.x = 0.0;
  base2camera.transform.translation.y = 0.0;
  base2camera.transform.translation.z = 5.0;
  base2camera.transform.rotation.x = 0.0;
  base2camera.transform.rotation.y = 0.0;
  base2camera.transform.rotation.z = 0.0;
  base2camera.transform.rotation.w = 1.0;

  // Broadcast static frame transformations
  br.sendTransform(base2robot);
  ROS_INFO_STREAM("Broadcasting base_link-->robot transform");
  br.sendTransform(base2camera);
  ROS_INFO_STREAM("Broadcasting base_link-->camera transform");

  ros::spinOnce();

  // Initialise Subscriber/Publisher nodes
  ros::Subscriber locationSub = node.subscribe("odometry/filtered", 1000, locationCallback);
  ros::Subscriber objectSub = node.subscribe("featurePositions/next", 1000, objectCallback);

  ros::Publisher targetPub = node.advertise<geometry_msgs::PoseStamped>("robotTarget", 1000);

  // Initialise robot target service
  //ros::ServiceServer service = node.advertiseService("send_target", send_target);
  //ROS_INFO("Ready to send target poses");

  ros::Rate loop_rate(1);  // Run at 10Hz

  while (ros::ok())
  {
    // Publish robot target
    try {
      geometry_msgs::PoseStamped targetPosition = getTargetPosition();
      targetPub.publish(targetPosition);
      ROS_INFO_STREAM("Publishing robot target location:\n" << targetPosition.pose);
    }
    catch (tf2::ExtrapolationException &e) {
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
};
