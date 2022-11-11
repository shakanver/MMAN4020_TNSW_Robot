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
bool usingGroundTruth = true;

const std::string OOI_LOCATION_FILE = "/home/shakeel/thesis/MMAN4020_TNSW_Robot/tf2_testgrounds/src/ooi_locations.txt";

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

//function prototypes
bool updateOperation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
void updateCurrTarget();
void readMapData();

bool updateOperation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{

  ROS_INFO_STREAM("Update Operation Triggered");

  operationCount++;
  
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
    ROS_INFO_STREAM("Current Operation is Candybar");
    res.message = "candybar";
    coneCount++;
  }
  else
  {
    ROS_INFO_STREAM("Current Operation is SpigotHole");
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
      ROS_INFO_STREAM("Doing data association on spigot hole");
      tf2::doTransform(foundSpigotLocation, temp, transform);
      
      for (int i = 0; i < spigotLocations.size(); i++)
      {
        ROS_INFO_STREAM("ground truth: " << spigotLocations[i][0] << " " << spigotLocations[i][1]);
        ROS_INFO_STREAM("found locations (global): "  << temp.x <<  " " << temp.y); 
        ROS_INFO_STREAM("found locations  (wrt base): " << foundSpigotLocation.x << " " << foundSpigotLocation.y);
        
        double distance = sqrt(pow(spigotLocations[i][0]-temp.x,2) + pow(spigotLocations[i][1]-temp.y,2));
            
        if (distance <= DISTANCE_THRESHOLD)
        {
            found = true;
            association = i;
            foundX = spigotLocations[i][0];
            foundY = spigotLocations[i][1];
            ROS_INFO_STREAM("Found spigot hole data association: " << foundX << " " << foundY << "\n");
            break;
        }
      }
    }
    // If operation is to pick up cone from spigot - we need to detect cone
    else if (operationOrder[operationCount] == 'c')
    {
      ROS_INFO_STREAM("Doing data association on candy bar");
      geometry_msgs::Point temp;
      tf2::doTransform(foundConeLocation, temp, transform);
      
      for (int i = 0; i < coneLocations.size(); i++)
      {
        double distance = sqrt(pow(coneLocations[i][0]-temp.x,2) + pow(coneLocations[i][1]-temp.y,2));
            
        if (distance <= DISTANCE_THRESHOLD)
        {
            found = true;
            association = i;
            foundX = coneLocations[i][0];
            foundY = coneLocations[i][1];
            break;
        }
      }
    }
    
    if (found)
    {
      ROS_INFO_STREAM("A target of type " << operationOrder[operationCount] << " was found and associated with the " << association << "the type known location");
      
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


void readMapData()
{
    ROS_INFO("Reading map data");
    
    std::ifstream fin;
    fin.open(OOI_LOCATION_FILE);

    std::string line, part, type;
    std::vector<std::string> row;
    double x, y;
    while (getline(fin, line))
    {
        row.clear();

        std::stringstream ss(line);
        ROS_INFO_STREAM("cur line: " << line);
        while (getline(ss, part, ','))
        {
            row.push_back(part);
        }


        type = row[0];
        x = stod(row[1]);
        y = stod(row[2]);

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
    ROS_INFO_STREAM("spigot locations: " << spigotLocations[0][0] << " " << spigotLocations[0][1]);
    ROS_INFO("Map data read");
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "data_associator");

  readMapData();

  // Initialise node information
  ros::NodeHandle node;
  
  tf2_ros::TransformListener listener(tf_buffer);

  ros::ServiceClient client = node.serviceClient<opencv_services::box_and_target_position>("box_and_target_position");

  opencv_services::box_and_target_position srv;
  
  ros::Publisher objectPub = node.advertise<geometry_msgs::Point>("featurePositions/next", 1000);
  
  // Initialise robot target update service
  ros::ServiceServer service = node.advertiseService("update_target", updateOperation);
  ROS_INFO("Ready to send target poses");

  // while (ros::ok() && operationCount == -1)
  // {
  //   ros::spinOnce();
  // }

  ros::Rate loop_rate(1);  // Run at 1Hz
  while (ros::ok())
  {
    // Update cone/spigot locations
    if (client.call(srv))
    {
      foundConeLocation = srv.response.box_position;
      foundSpigotLocation = srv.response.target_position;
      std::cout << "spigot hole location: " << foundSpigotLocation.x << " " << foundSpigotLocation.y << "\n";
    }
    else
    {
      ROS_ERROR("Failed to call service to update target locations");
      return 1;
    }

    std::cout << "Curr target being published to tf2 node: " << "\n" << currTarget << "\n";
    objectPub.publish(currTarget);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
};
