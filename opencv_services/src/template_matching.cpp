 /*
  * OpenCV Example using ROS and CPP
  */

 // Include the ROS library
 #include <ros/ros.h>

 // Include opencv2
 #include <opencv2/core/mat.hpp>
 #include <opencv2/highgui.hpp>
 #include <opencv2/imgproc.hpp>


 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <sensor_msgs/PointCloud2.h>
 #include <geometry_msgs/Point.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <geometry_msgs/Pose.h>

// Include tf2 for transformation
 #include <tf2_ros/buffer.h>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

 #include "opencv_services/box_and_target_position.h"

 // Topics
 static const std::string IMAGE_TOPIC = "/camera1/rgb/image_raw";
 static const std::string POINT_CLOUD2_TOPIC = "/camera1/depth/points";

 // Publisher
 ros::Publisher pub;

tf2_ros::Buffer tf_buffer;

const std::string from_frame = "camera_depth_optical_frame";
const std::string to_frame = "base_link";

cv::Mat camera_image;

cv::Point2f cone_centroid;
cv::Point2f spigot_centroid;

geometry_msgs::Point box_position_base_frame;
geometry_msgs::Point target_position_base_frame;



void apply_cv_algorithms(cv::Mat camera_image) {

  cv::Point top_left;
  cv::Point bottom_right;
  cv::Mat cone_result;
  cv::Mat spigot_result;
  cv::Point2f cone_pos;
  cv::Point2f spigot_pos;
  cv::Scalar color = cv::Scalar(0,0,255); // B G R values

  cv::namedWindow( "Camera Image", cv::WINDOW_AUTOSIZE );

  // Draw bounding box around cone
  cv::Mat cone_img;
  cone_img = cv::imread("/home/shakeel/thesis/MMAN4020_TNSW_Robot/opencv_services/src/cone_template.png", cv::IMREAD_COLOR);
  // cv::TM_CCOEFF_NORMED = 5
  cv::matchTemplate(camera_image, cone_img, cone_result, 5);    // cv::TM_CCOEFF_NORMED = 5
  // normalize( cone_result, cone_result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
  double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
  minMaxLoc( cone_result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
  // set gloabl variable cone_centroid
  cone_centroid = cv::Point2f(maxLoc.x + cone_img.cols/2.0, maxLoc.y + cone_img.rows/2.0);
  cv::rectangle( camera_image, maxLoc, cv::Point( maxLoc.x + cone_img.cols , maxLoc.y + cone_img.rows ), cv::Scalar::all(0), 2, 8, 0 );
  cv::circle(camera_image, cone_centroid, 5, color, -1);

  cv::imshow( "Camera Image", camera_image );


  // Draw bounding box around spigot hole
  cv::Mat spigot_hole_img;
  spigot_hole_img = cv::imread("/home/shakeel/thesis/MMAN4020_TNSW_Robot/opencv_services/src/new_spigot_hole.png", cv::IMREAD_COLOR);
  cv::matchTemplate(camera_image, spigot_hole_img, spigot_result, 5);    // cv::TM_CCOEFF_NORMED = 5
  // normalize( spigot_result, spigot_result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
  minMaxLoc( spigot_result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
  cv::rectangle( camera_image, maxLoc, cv::Point( maxLoc.x + spigot_hole_img.cols , maxLoc.y + spigot_hole_img.rows ), cv::Scalar::all(0), 2, 8, 0 );
  cv::circle(camera_image, spigot_centroid, 5, color, -1);

  cv::imshow( "Camera Image", camera_image );

  cv::waitKey(3);
  // set gloabl variable spigot_centroid
  spigot_centroid = cv::Point2f(maxLoc.x + spigot_hole_img.cols/2.0, maxLoc.y + spigot_hole_img.rows/2.0);

  
  ROS_INFO_STREAM("2D cone position pixel coords: x " << cone_centroid.x << " y " << cone_centroid.y);
  ROS_INFO_STREAM("2D spigot position pixel coords: x " << spigot_centroid.x << " y " << spigot_centroid.y);


}



void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  float image_size_y = cv_ptr->image.rows;
  float image_size_x = cv_ptr->image.cols;

  apply_cv_algorithms(cv_ptr->image);

}


geometry_msgs::Point pixel_to_3d_point(const sensor_msgs::PointCloud2 pCloud, const int u, const int v)
{
  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  geometry_msgs::Point p;
  p.x = X;
  p.y = Y;
  p.z = Z;

  return p;
}

geometry_msgs::Point transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame) {
    
  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.pose.position = p;
  input_pose_stamped.header.frame_id = from_frame;
  input_pose_stamped.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped output_pose_stamped = tf_buffer.transform(input_pose_stamped, to_frame, ros::Duration(1));
  return output_pose_stamped.pose.position;
}

void point_cloud_cb(const sensor_msgs::PointCloud2 pCloud) {

  geometry_msgs::Point cone_position_camera_frame;
  cone_position_camera_frame = pixel_to_3d_point(pCloud, cone_centroid.x, cone_centroid.y);

  geometry_msgs::Point spigot_position_camera_frame;
  spigot_position_camera_frame = pixel_to_3d_point(pCloud, spigot_centroid.x, spigot_centroid.y);
    
  box_position_base_frame = transform_between_frames(cone_position_camera_frame, from_frame, to_frame);
  target_position_base_frame = transform_between_frames(spigot_position_camera_frame, from_frame, to_frame);

  ROS_INFO_STREAM("3d conw position base frame: x " << box_position_base_frame.x << " y " << box_position_base_frame.y << " z " << box_position_base_frame.z);
  ROS_INFO_STREAM("3d spigot position base frame: x " << target_position_base_frame.x << " y " << target_position_base_frame.y << " z " << target_position_base_frame.z);
}

// service call response
bool get_box_and_target_position(opencv_services::box_and_target_position::Request  &req,
    opencv_services::box_and_target_position::Response &res) {
      res.box_position = box_position_base_frame;
      res.target_position = target_position_base_frame;
      return true;
    }

 // Main function
int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "opencv_services");
   
  // Default handler for nodes in ROS
  ros::NodeHandle nh("");

    // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);

    // Subscribe to the /camera raw image topic
  image_transport::Subscriber image_sub = it.subscribe(IMAGE_TOPIC, 1, image_cb);

    // Subscribe to the /camera PointCloud2 topic
  ros::Subscriber point_cloud_sub = nh.subscribe(POINT_CLOUD2_TOPIC, 1, point_cloud_cb);
  
  tf2_ros::TransformListener listener(tf_buffer);
   
  ros::ServiceServer service = nh.advertiseService("box_and_target_position",  get_box_and_target_position);
   
  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();
   
  // Close down OpenCV
  cv::destroyWindow("view");
}

 

