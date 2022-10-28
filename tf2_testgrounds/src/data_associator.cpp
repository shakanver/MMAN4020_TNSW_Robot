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

#define DISTANCE_THRESHOLD 0.05

const std::string OOI_LOCATION_FILE = "/home/neoblivion/integration_testing/ooi_locations.csv";

int operationCount = -1;
int spigotCount = -1;
int coneCount = -1;
char operationOrder[] = {'s','c'};

tf2_ros::Buffer tf_buffer;

std::vector<std::array<double,2>> coneLocations;      // x, y
std::vector<std::array<double,2>> spigotLocations;    // x, y

geometry_msgs::Point currTarget;

std::vector<std::array<double,3>> foundConeLocations;       // x, y, association (-1 for none)
std::vector<std::array<double,3>> foundSpigotLocations;     

bool update_target(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  operationCount++;

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

  ROS_INFO_STREAM("Updated target to next " << res.message << " for operation No." << operationCount);

  return true;
}

void cameraConeCallback(geometry_msgs::PoseArray msg)
{
  //ROS_INFO_STREAM("Received candybar location data");

  foundConeLocations.clear();

  // Assume locations are in camera frame - transform to map frame
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer.lookupTransform("map", "camera", ros::Time(0));

    for (geometry_msgs::Pose p : msg.poses)
    {
        // Transform data
        geometry_msgs::Point temp;
        tf2::doTransform(p.position, temp, transform);

        // Associate data
        double association = -1;
        for (int i = 0; i < coneLocations.size(); i++)
        {
            double distance = sqrt(pow(coneLocations[i][0]-temp.x,2) + pow(coneLocations[i][1]-temp.y,2));
            
            if (distance <= DISTANCE_THRESHOLD)
            {
                association = i;
                break;
            }
        }

        foundConeLocations.push_back({temp.x,temp.y,association});

        // Determine pose estimates - will need to send as individual poses to localisation node
    }
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void cameraSpigotCallback(geometry_msgs::PoseArray msg)
{
  //ROS_INFO_STREAM("Received spigot location data");

  foundSpigotLocations.clear();

  // Assume locations are in camera frame - transform to map frame
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer.lookupTransform("map", "camera", ros::Time(0));
    
    for (geometry_msgs::Pose p : msg.poses)
    {
        // Transform data
        geometry_msgs::Point temp;
        tf2::doTransform(p.position, temp, transform);

        // Associate data
        double association = -1;
        for (int i = 0; i < spigotLocations.size(); i++)
        {
            double distance = sqrt(pow(spigotLocations[i][0]-temp.x,2) + pow(spigotLocations[i][1]-temp.y,2));

            if (distance <= DISTANCE_THRESHOLD)
            {
                association = i;
                break;
            }
        }

        foundSpigotLocations.push_back({temp.x,temp.y,association});

        // Determine pose estimates - will need to send as individual poses to localisation node
    }
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

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

  ros::Subscriber cameraConeSub = node.subscribe("conePositions/camera", 1000, cameraConeCallback);
  ros::Subscriber cameraSpigotSub = node.subscribe("spigotPositions/camera", 1000, cameraSpigotCallback);
  
  ros::Publisher objectPub = node.advertise<geometry_msgs::Point>("featurePositions/next", 1000);
  
  // Initialise robot target update service
  ros::ServiceServer service = node.advertiseService("update_target", update_target);
  ROS_INFO("Ready to send target poses");

  while (ros::ok() && operationCount == -1)
  {
    ros::spinOnce();
  }

  ros::Rate loop_rate(1);  // Run at 1Hz
  while (ros::ok())
  {
    geometry_msgs::Point targetFeature;
    bool found = false;

    // Publish feature located by camera (for demonstration purposes) associated to feature
    if (operationOrder[operationCount] == 'c')
    {
        for (int i = 0; i < foundConeLocations.size(); i++)
        {
            if (foundConeLocations[i][2] == coneCount)
            {
                targetFeature.x = foundConeLocations[i][0];
                targetFeature.y = foundConeLocations[i][1];

                found = true;
                break;
            }
        }
    }
    else
    {
        for (int i = 0; i < foundSpigotLocations.size(); i++)
        {
            if (foundSpigotLocations[i][2] == spigotCount)
            {
                targetFeature.x = foundSpigotLocations[i][0];
                targetFeature.y = foundSpigotLocations[i][1];

                found = true;
                break;
            }
        }
    }

    if (found)
    {
        objectPub.publish(targetFeature);
        //ROS_INFO_STREAM("Publishing target feature location of type " << operationOrder[operationCount]);
    }
    else {
        ROS_INFO("An associated landmark was not found by the camera");
        ROS_INFO_STREAM(operationCount << " " << coneCount << " " << spigotCount);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
};