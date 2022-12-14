#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <vector>
#include <fstream>
#include "opencv_services/box_and_target_position.h"

#define DISTANCE_THRESHOLD 0.05

// Are we directly forwarding camera data or using ground truh from map?
bool usingGroundTruth = false;

const std::string OOI_LOCATION_FILE = "/home/neoblivion/integration_testing/ooi_locations.csv";

int operationCount = -1;
int spigotCount = -1;
int coneCount = -1;
char operationOrder[] = {'s','c'};

tf2_ros::Buffer tf_buffer;

std::vector<std::array<double,2>> coneLocations;      // x, y
std::vector<std::array<double,2>> spigotLocations;    // x, y

geometry_msgs::Point currTarget;

geometry_msgs::Point foundConeLocation;
geometry_msgs::Point foundSpigotLocation;

opencv_services::box_and_target_position srv;

bool updateOperation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  operationCount++;
  
  // Update cone/spigot locations
  if (client.call(srv))
  {
    geometry_msgs::Point foundConeLocation = srv.response.box_position;
    geometry_msgs::Point foundSpigotLocation = srv.response.target_position;
  }
  else
  {
    ROS_ERROR("Failed to call service to update target locations");
    return 1;
  }
  
  // Check if we're a the end of operation
  if (operationCount >= sizeof(operationOrder) / sizeof(char))
  {
    res.success = true;
    res.message = "STOP PLZ";
    ROS_INFO("End of operation reached");

    return true;
  }

  if (operationOrder[operationCount] == 'c')
  {
    res.message = "candybar";
    coneCount++;
  }
  else
  {
    res.message = "spigot";
    spigotCount++;
  }

  res.success = true;
  
  // Update current operation target - given in map frame
  updateCurrTarget();

  ROS_INFO_STREAM("Updated target to next " << res.message << " for operation No." << operationCount);

  return true;
}

void updateCurrTarget()
{
  ROS_INFO_STREAM("Received candybar location data");
  
  // Locations provided are in robot base frame
  
  // Assume locations are in camera frame - transform to map frame
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
    
    bool found = false;
    int association = -1;
    double foundX, foundY = 0;
    geometry_msgs::Point temp;
    // If operation is to place cone in spigot - we need to detect spigot
    if (operationOrder[operationCount] == 's')
    {
      tf2::doTransform(foundSpigotLocation, temp, transform);
      
      for (int i = 0; i < spigotLocations.size(); i++)
      {
        double distance = sqrt(pow(spigotLocations[i][0]-temp.x,2) + pow(spigotLocations[i][1]-temp.y,2));
            
        if (distance <= DISTANCE_THRESHOLD)
        {
            found = true;
            association = i;
            foundX = spigotLocations[i][0];
            foundY = spigotLocations[i][1];
            break;
        }
      }
    }
    // If operation is to pick up cone from spigot - we need to detect cone
    else if (operationOrder[operationCount] == 'c')
    {
      geometry_msgs::Point temp;
      tf2::doTransform(foundConeLocation, temp, transform);
      
      for (int i = 0; i < coneLocations.size(); i++)
      {
        double distance = sqrt(pow(coneLocations[i][0]-temp.x,2) + pow(conetLocations[i][1]-temp.y,2));
            
        if (distance <= DISTANCE_THRESHOLD)
        {
            found = true;
            association = i;
            foundX = spigotLocations[i][0];
            foundY = spigotLocations[i][1];
            break;
        }
      }
    }
    
    if (found)
    {
      ROS_INFO_STREAM("A target of type " << operationOrder[operationCount] << " was found and associated with the " << association << "th type known location");
      
      if (usingGroundTruth)
      {
        currTarget.x = foundX;
        currTarget.y = foundY;
      }
      else
      {
        currTarget = temp;
      }
      
      ROS_INFO_STREAM("Current target being updated to x: " << currTarget.x << " y: " << currTarget.y << " z: " << currTarget.z);
    }
    else
    {
        ROS_INFO("An associated landmark was not found by the camera");
        // Set the currTarget to something else?
    }
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

// void cameraConeCallback(geometry_msgs::PoseArray msg)
// {
//   //ROS_INFO_STREAM("Received candybar location data");

//   foundConeLocations.clear();

//   // Assume locations are in camera frame - transform to map frame
//   geometry_msgs::TransformStamped transform;
//   try {
//     transform = tf_buffer.lookupTransform("map", "camera", ros::Time(0));

//     for (geometry_msgs::Pose p : msg.poses)
//     {
//         // Transform data
//         geometry_msgs::Point temp;
//         tf2::doTransform(p.position, temp, transform);

//         // Associate data
//         double association = -1;
//         for (int i = 0; i < coneLocations.size(); i++)
//         {
//             double distance = sqrt(pow(coneLocations[i][0]-temp.x,2) + pow(coneLocations[i][1]-temp.y,2));
            
//             if (distance <= DISTANCE_THRESHOLD)
//             {
//                 association = i;
//                 break;
//             }
//         }

//         foundConeLocations.push_back({temp.x,temp.y,association});

//         // Determine pose estimates - will need to send as individual poses to localisation node
//     }
//   }
//   catch (tf2::TransformException &ex) {
//     ROS_WARN("%s", ex.what());
//     ros::Duration(1.0).sleep();
//   }
// }

// void cameraSpigotCallback(geometry_msgs::PoseArray msg)
// {
//   //ROS_INFO_STREAM("Received spigot location data");

//   foundSpigotLocations.clear();

//   // Assume locations are in camera frame - transform to map frame
//   geometry_msgs::TransformStamped transform;
//   try {
//     transform = tf_buffer.lookupTransform("map", "camera", ros::Time(0));
    
//     for (geometry_msgs::Pose p : msg.poses)
//     {
//         // Transform data
//         geometry_msgs::Point temp;
//         tf2::doTransform(p.position, temp, transform);

//         // Associate data
//         double association = -1;
//         for (int i = 0; i < spigotLocations.size(); i++)
//         {
//             double distance = sqrt(pow(spigotLocations[i][0]-temp.x,2) + pow(spigotLocations[i][1]-temp.y,2));

//             if (distance <= DISTANCE_THRESHOLD)
//             {
//                 association = i;
//                 break;
//             }
//         }

//         foundSpigotLocations.push_back({temp.x,temp.y,association});

//         // Determine pose estimates - will need to send as individual poses to localisation node
//     }
//   }
//   catch (tf2::TransformException &ex) {
//     ROS_WARN("%s", ex.what());
//     ros::Duration(1.0).sleep();
//   }
// }

void readMapData()
{
    // char temp[100];
    // getcwd(temp, sizeof(temp));
    // ROS_INFO_STREAM(temp);
    ROS_INFO("Reading map data");
    
    std::ifstream fin;
    // fin.open("ooi_locations.csv");
    fin.open(OOI_LOCATION_FILE);

    std::string line, part, type;
    std::vector<std::string> row;
    double x, y;
    while (getline(fin, line))
    {
        row.clear();

        std::stringstream ss(line);

        while (getline(ss, part, ','))
        {
            row.push_back(part);
        }

        type = row[0];
        x = stod(row[1]);
        y = stod(row[2]);
        
        ROS_INFO_STREAM(type << ": " << x << " " << y);

        if (type == "c")
        {
            coneLocations.push_back({x,y});
            ROS_INFO_STREAM("c " << x << " " << y);
        }
        else if (type == "s")
        {
            spigotLocations.push_back({x,y});
            ROS_INFO_STREAM("s " << x << " " << y);
        }
    }

    ROS_INFO("Map data read");
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "data_associator");

  readMapData();

  // Initialise node information
  ros::NodeHandle node;
  
  tf2_ros::TransformListener listener(tf_buffer);

  //   ros::Subscriber cameraConeSub = node.subscribe("conePositions/camera", 1000, cameraConeCallback);
  //   ros::Subscriber cameraSpigotSub = node.subscribe("spigotPositions/camera", 1000, cameraSpigotCallback);
  ros::ServiceClient client = node.serviceClient<opencv_services::box_and_target_position>("box_and_target_position");

  //opencv_services::box_and_target_position srv;
  
  ros::Publisher objectPub = node.advertise<geometry_msgs::Point>("featurePositions/next", 1000);
  
  // Initialise robot target update service
  ros::ServiceServer service = node.advertiseService("update_target", updateOperation);
  ROS_INFO("Ready to send target poses");

  while (ros::ok() && operationCount == -1)
  {
    ros::spinOnce();
  }

  ros::Rate loop_rate(1);  // Run at 1Hz
  while (ros::ok())
  {
//     geometry_msgs::Point targetFeature;
//     bool found = false;

//     // Publish feature located by camera (for demonstration purposes) associated to feature
//     if (operationOrder[operationCount] == 'c')
//     {
//         for (int i = 0; i < foundConeLocations.size(); i++)
//         {
//             if (foundConeLocations[i][2] == coneCount)
//             {
//                 targetFeature.x = foundConeLocations[i][0];
//                 targetFeature.y = foundConeLocations[i][1];

//                 found = true;
//                 break;
//             }
//         }
//     }
//     else
//     {
//         for (int i = 0; i < foundSpigotLocations.size(); i++)
//         {
//             if (foundSpigotLocations[i][2] == spigotCount)
//             {
//                 targetFeature.x = foundSpigotLocations[i][0];
//                 targetFeature.y = foundSpigotLocations[i][1];

//                 found = true;
//                 break;
//             }
//         }
//     }

//     if (found)
//     {
//         objectPub.publish(targetFeature);
//         //ROS_INFO_STREAM("Publishing target feature location of type " << operationOrder[operationCount]);
//     }
//     else {
//         ROS_INFO("An associated landmark was not found by the camera");
//         ROS_INFO_STREAM(operationCount << " " << coneCount << " " << spigotCount);
//     }
    
    objectPub.publish(currTarget);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
};
