#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  geometry_msgs::PoseArray coneLocationsCamera;
  geometry_msgs::PoseArray spigotLocationsCamera;
  nav_msgs::Odometry robotLocationOdom;

  geometry_msgs::TransformStamped map2odom;  // Constant as both are fixed
  geometry_msgs::TransformStamped odom2base;
  
  ros::init(argc, argv, "fake_publisher");

  ros::NodeHandle node;
  // Publish odom --> base_link transform
  // Publish odom pos
  
  ros::Publisher locationPub = node.advertise<nav_msgs::Odometry>("odometry/filtered", 1000);
//   ros::Publisher conePub = node.advertise<geometry_msgs::PoseArray>("conePositions/camera", 1000);
//   ros::Publisher spigPub = node.advertise<geometry_msgs::PoseArray>("spigotPositions/camera", 1000);

  tf2_ros::TransformBroadcaster br;

  ros::Rate loop_rate(1);  // Run at 10Hz

  while (ros::ok())
  {
    // Update parameters
//     geometry_msgs::Pose temp;
//     temp.position.x = 2.0;
//     temp.position.y = 7.0;
//     temp.position.z = 0.0;
//     geometry_msgs::Pose temp2;
//     temp2.position.x = 10.01;
//     temp2.position.y = 9.96;
//     temp2.position.z = 0.0;
//     coneLocationsCamera.header.stamp = ros::Time::now();
//     coneLocationsCamera.header.frame_id = "camera";
//     coneLocationsCamera.poses = {temp, temp2};

//     temp.position.x = -0.02;
//     temp.position.y = 5.03;
//     temp.position.z = 0.0;
//     temp2.position.x = 10.01;
//     temp2.position.y = 9.95;
//     temp2.position.z = 0.0;
//     spigotLocationsCamera.header.stamp = ros::Time::now();
//     spigotLocationsCamera.header.frame_id = "camera";
//     spigotLocationsCamera.poses = {temp, temp2};

    robotLocationOdom.header.stamp = ros::Time::now();
    robotLocationOdom.header.frame_id = "odom";
    robotLocationOdom.child_frame_id = "base_link";
    robotLocationOdom.pose.pose.position.x = 0.0;
    robotLocationOdom.pose.pose.position.y = 0.0;
    robotLocationOdom.pose.pose.position.z = 0.0;
    robotLocationOdom.pose.pose.orientation.x = 0.0;
    robotLocationOdom.pose.pose.orientation.y = 0.0;
    robotLocationOdom.pose.pose.orientation.z = 0.0;
    robotLocationOdom.pose.pose.orientation.w = 1.0;
    
    map2odom.header.stamp = ros::Time::now();
    map2odom.header.frame_id = "map";
    map2odom.child_frame_id = "odom";
    map2odom.transform.translation.x = 0.0;
    map2odom.transform.translation.y = 0.0;
    map2odom.transform.translation.z = 0.0;
    map2odom.transform.rotation.x = 0.0;
    map2odom.transform.rotation.y = 0.0;
    map2odom.transform.rotation.z = 0.0;
    map2odom.transform.rotation.w = 1.0;

    odom2base.header.stamp = ros::Time::now();
    odom2base.header.frame_id = "odom";
    odom2base.child_frame_id = "base_link";
    odom2base.transform.translation.x = 0.0;
    odom2base.transform.translation.y = 0.0;
    odom2base.transform.translation.z = 0.0;
    odom2base.transform.rotation.x = 0.0;
    odom2base.transform.rotation.y = 0.0;
    odom2base.transform.rotation.z = 0.0;
    odom2base.transform.rotation.w = 1.0;

    // Publish
    br.sendTransform(map2odom);
    ROS_INFO_STREAM("Broadcasting map-->odom transform");
    br.sendTransform(odom2base);
    ROS_INFO_STREAM("Broadcasting odom-->base_link transform");
    locationPub.publish(robotLocationOdom);
    ROS_INFO_STREAM("Publishing robot location");
//     conePub.publish(coneLocationsCamera);
//     ROS_INFO_STREAM("Publishing candy bar locations");
//     spigPub.publish(spigotLocationsCamera);
//     ROS_INFO_STREAM("Publishing spigot locations");

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
};

