#include <ros/ros.h>
//#include <tf2_testgrounds/SendTarget.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "fake_client");

  ros::NodeHandle node;

  ros::ServiceClient client = node.serviceClient<std_srvs::Trigger>("update_target");
  std_srvs::Trigger srv;

  if (client.call(srv))
  {
    bool success = srv.response.success;
    std::string message = srv.response.message;

    ROS_INFO_STREAM("Success: " << srv.response.success << " | " << srv.response.message);
  }
  else
  {
    ROS_ERROR("Failed to call service send_target");
    return 1;
  }

  // ros::ServiceClient client = node.serviceClient<tf2_testgrounds::SendTarget>("send_target");

  // tf2_testgrounds::SendTarget srv;
  // srv.request.a = 13;

  // if (client.call(srv))
  // {
  //   geometry_msgs::Pose tPose = srv.response.targetPose;

  //   ROS_INFO_STREAM("x: " << tPose.position.x << "y: " << tPose.position.y << "z: " << tPose.position.z << "\n");
  //   ROS_INFO_STREAM("x: " << tPose.orientation.x << "y: " << tPose.orientation.y << "z: " << tPose.orientation.z << "w: " << tPose.orientation.w << "\n");
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service send_target");
  //   return 1;
  // }
  
  return 0;
};