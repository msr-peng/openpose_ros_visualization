// This code synchronizes the 3d point cloud and 2D image and publishes 3d locations of human skeletons
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <openpose_ros_msgs/GetPersons.h>

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

#include <vector>
#include <cmath>

#include <openpose_ros_msgs/BodypartDetection_3d.h>
#include <openpose_ros_msgs/PersonDetection_3d.h>


// Set resolution

#define width  640
#define height 480

// Declare Publishers
ros::Publisher           pc_pub;
ros::Publisher           image_pub;   

// Declare 3d keypoints publisher
ros::Publisher           keypoints_3d_pub;

// Function to initialize all skeletal detections

openpose_ros_msgs::BodypartDetection_3d getNANBodypart()
{
  openpose_ros_msgs::BodypartDetection_3d bodypart_depth;
  bodypart_depth.x = NAN;
  bodypart_depth.y = NAN;
  bodypart_depth.z = NAN;
  bodypart_depth.confidence = NAN;
  return bodypart_depth;
}

// Function for when a bodypart is not detected
void notDetectedBodyPart(std::string bodypart_name)
{
  std::cerr << bodypart_name << " not detected!!" << std::endl;

  std::cerr << bodypart_name << " pixel coordinates (x,y,z): " << std::endl;
  std::cerr << "( "<< "nan" << ", " 
               << "nan" << ", "
                   << "nan" << ")" << std::endl;

  std::cerr << bodypart_name << " real world coordinates (x,y,z): " << std::endl;
  std::cerr << "( " << "nan" << ", " 
                << "nan" << ", "
                    << "nan" << ")" << std::endl;
}

// Function to get 3d detections

openpose_ros_msgs::BodypartDetection_3d get3dcoordinates(const openpose_ros_msgs::BodypartDetection bodypart_2d, const pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud, const std::string bodypart_name)
{
  openpose_ros_msgs::BodypartDetection_3d bodypart_depth;

  // Include 2d confidence 3d detections message
  bodypart_depth.confidence = bodypart_2d.confidence;

  // If not detected bodypart
  if (std::isnan(bodypart_depth.confidence) || bodypart_depth.confidence == 0.0 || (bodypart_2d.x == 0 && bodypart_depth.y == 0) || bodypart_2d.x > width || bodypart_2d.y > height )
  {
    // notDetectedBodyPart(bodypart_name);
    bodypart_depth.x = NAN;
    bodypart_depth.y = NAN;
    bodypart_depth.z = NAN;
  }

  // If detected bodypart
  else
  {
    // Get keypoint pixel coordinates
    unsigned long long int x_pixel = bodypart_2d.x;
    unsigned long long int y_pixel = bodypart_2d.y;

    bodypart_depth.x = temp_cloud->points[width * y_pixel + x_pixel].x;
    bodypart_depth.y = temp_cloud->points[width * y_pixel + x_pixel].y;
    bodypart_depth.z = temp_cloud->points[width * y_pixel + x_pixel].z;
  }
  return bodypart_depth;
}
// Declare Service Client
ros::ServiceClient client;
openpose_ros_msgs::GetPersons srv;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

//Declare Callback
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& image_msg)
{
  ROS_INFO("Cloud and Image Messages Received!");
  ROS_INFO("    Cloud Time Stamp: %f", cloud_msg->header.stamp.toSec());
  ROS_INFO("    Image Time Stamp: %f", image_msg->header.stamp.toSec()); 


  // Publish input pointcloud and image
  pc_pub.publish(*cloud_msg);
  image_pub.publish(*image_msg); 
 
  srv.request.image = *image_msg;
  if (client.call(srv))
  {
    ROS_INFO("ROS Service call Successful");
    // Prepare a new ROS Message for all skeletal detections
    // Initialize message for skeletal detections
    openpose_ros_msgs::PersonDetection_3d person_msg;

    // Number of people detected
    int num_people = srv.response.detections.size();

    // Number of bodyparts, we suppose we are working with COCO
    int num_bodyparts = 25;

    // for each detection (person),
    for (size_t person_idx = 0; person_idx < num_people; person_idx++)
    {
      // Prepare a new ROS Message for this skeletal detection
      // Add number of people detected
      person_msg.num_people_detected = num_people;
      // Add person ID
      person_msg.person_ID = person_idx;
      // Get corresponding 2d skeleton detection for person with ID idx
      openpose_ros_msgs::PersonDetection skeleton_detections = srv.response.detections[person_idx];
      // Initialize all bodyparts (x,y,z=0 & confidence = nan)
      person_msg.nose = getNANBodypart();
      person_msg.neck = getNANBodypart();
      person_msg.right_shoulder = getNANBodypart();
      person_msg.right_elbow = getNANBodypart();
      person_msg.right_wrist = getNANBodypart();
      person_msg.left_shoulder = getNANBodypart();
      person_msg.left_elbow = getNANBodypart();
      person_msg.left_wrist = getNANBodypart();
      person_msg.mid_hip = getNANBodypart();
      person_msg.right_hip = getNANBodypart();
      person_msg.right_knee = getNANBodypart();
      person_msg.right_ankle = getNANBodypart();
      person_msg.left_hip = getNANBodypart();
      person_msg.left_knee = getNANBodypart();
      person_msg.left_ankle = getNANBodypart();
      person_msg.right_eye = getNANBodypart();
      person_msg.left_eye = getNANBodypart();
      person_msg.right_ear = getNANBodypart();
      person_msg.left_ear = getNANBodypart();
      person_msg.left_big_toe = getNANBodypart();
      person_msg.left_small_toe = getNANBodypart();
      person_msg.left_heel = getNANBodypart();
      person_msg.right_big_toe = getNANBodypart();
      person_msg.right_small_toe = getNANBodypart();
      person_msg.right_heel = getNANBodypart();

      // Declare pcl<xyz> pointcloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

      //Load pointcloud data from cloud_msg (sensor_msgs::Pointcloud2) to temp_cloud of type pcl::Pointcloud<pcl::PointXYZ>
      pcl::fromROSMsg (*cloud_msg, *temp_cloud);

      // for each body part 
      for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++)
      {
        // Initialize bodypart msg 
        openpose_ros_msgs::BodypartDetection bodypart_detections;
        switch (bodypart_idx)
        {
          case 0: bodypart_detections = skeleton_detections.nose;
                  person_msg.nose = get3dcoordinates(bodypart_detections, temp_cloud, "Nose"); break;
          case 1: bodypart_detections = skeleton_detections.neck;
                  person_msg.neck = get3dcoordinates(bodypart_detections, temp_cloud, "Neck"); break;
          case 2: bodypart_detections = skeleton_detections.right_shoulder;
                  person_msg.right_shoulder = get3dcoordinates(bodypart_detections, temp_cloud, "RShoulder"); break;
          case 3: bodypart_detections = skeleton_detections.right_elbow;
                  person_msg.right_elbow = get3dcoordinates(bodypart_detections, temp_cloud, "RElbow"); break;
          case 4: bodypart_detections = skeleton_detections.right_wrist;
                  person_msg.right_wrist = get3dcoordinates(bodypart_detections, temp_cloud, "RWrist"); break;
          case 5: bodypart_detections = skeleton_detections.left_shoulder;
                  person_msg.left_shoulder = get3dcoordinates(bodypart_detections, temp_cloud, "LShoulder"); break;
          case 6: bodypart_detections = skeleton_detections.left_elbow;
                  person_msg.left_elbow = get3dcoordinates(bodypart_detections, temp_cloud, "LElbow"); break;
          case 7: bodypart_detections = skeleton_detections.left_wrist;
                  person_msg.left_wrist = get3dcoordinates(bodypart_detections, temp_cloud, "LWrist"); break;
          case 8: bodypart_detections = skeleton_detections.mid_hip;
                  person_msg.mid_hip = get3dcoordinates(bodypart_detections, temp_cloud, "MidHip"); break;
          case 9: bodypart_detections = skeleton_detections.right_hip;
                  person_msg.right_hip = get3dcoordinates(bodypart_detections, temp_cloud, "RHip"); break;
          case 10:bodypart_detections = skeleton_detections.right_knee;
                  person_msg.right_knee = get3dcoordinates(bodypart_detections, temp_cloud, "RKnee"); break;
          case 11:bodypart_detections = skeleton_detections.right_ankle;
                  person_msg.right_ankle = get3dcoordinates(bodypart_detections, temp_cloud, "RAnkle"); break;
          case 12:bodypart_detections = skeleton_detections.left_hip;
                  person_msg.left_hip = get3dcoordinates(bodypart_detections, temp_cloud, "LHip"); break;
          case 13:bodypart_detections = skeleton_detections.left_knee;
                  person_msg.left_knee = get3dcoordinates(bodypart_detections, temp_cloud, "LKnee"); break;
          case 14:bodypart_detections = skeleton_detections.left_ankle;
                  person_msg.left_ankle = get3dcoordinates(bodypart_detections, temp_cloud, "LAnkle"); break;
          case 15:bodypart_detections = skeleton_detections.right_eye;
                  person_msg.right_eye = get3dcoordinates(bodypart_detections, temp_cloud, "REye"); break;
          case 16:bodypart_detections = skeleton_detections.left_eye;
                  person_msg.left_eye = get3dcoordinates(bodypart_detections, temp_cloud, "LEye"); break;
          case 17:bodypart_detections = skeleton_detections.right_ear;
                  person_msg.right_ear = get3dcoordinates(bodypart_detections, temp_cloud, "REar"); break;
          case 18:bodypart_detections = skeleton_detections.left_ear;
                  person_msg.left_ear = get3dcoordinates(bodypart_detections, temp_cloud, "LEar"); break;
          case 19:bodypart_detections = skeleton_detections.left_big_toe;
                  person_msg.left_big_toe = get3dcoordinates(bodypart_detections, temp_cloud, "LBigToe"); break;
          case 20:bodypart_detections = skeleton_detections.left_small_toe;
                  person_msg.left_small_toe = get3dcoordinates(bodypart_detections, temp_cloud, "LSmallToe"); break;
          case 21:bodypart_detections = skeleton_detections.left_heel;
                  person_msg.left_heel = get3dcoordinates(bodypart_detections, temp_cloud, "LHeel"); break;
          case 22:bodypart_detections = skeleton_detections.right_big_toe;
                  person_msg.right_big_toe = get3dcoordinates(bodypart_detections, temp_cloud, "RBigToe"); break;
          case 23:bodypart_detections = skeleton_detections.right_small_toe;
                  person_msg.right_small_toe = get3dcoordinates(bodypart_detections, temp_cloud, "RSmallToe"); break;
          case 24:bodypart_detections = skeleton_detections.right_heel;
                  person_msg.right_heel = get3dcoordinates(bodypart_detections, temp_cloud, "RHeel"); break;
        }
      }
      // Publish 3D detection
      keypoints_3d_pub.publish(person_msg);
    }
  }
  else
  {
    ROS_INFO("test message");
    ROS_ERROR("Failed to call service detect_poses_3d");
    ROS_ERROR("Did you remap the service and client names?");
    ROS_ERROR("This node expects a service called skeleton_2d_detector in which a launch file should have handled the remapping");
  }
}

int main(int argc, char** argv){
  // Initialize ROS
  ros::init(argc, argv, "skeleton_extract_3d_node");
  ROS_INFO("test message");

  // Declare Node Handle
  ros::NodeHandle nh("~");

  // Declare Subscribers
  // Synchronize Point Cloud and Image Subscription Received Messages
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/depth_registered/points", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_rect_color", 1);
  client = nh.serviceClient<openpose_ros_msgs::GetPersons>("/detect_poses_3d");


  // Pointcloud publisher topic /openpose_ros/input_pointcloud
  pc_pub = nh.advertise<sensor_msgs::PointCloud2>( "/openpose_ros/skeleton_3d/input_pointcloud", 0);

  // Image publisher topic /openpose_ros/input_rgb
  image_pub = nh.advertise<sensor_msgs::Image>( "/openpose_ros/skeleton_3d/input_rgb", 0);

  // Keypoints in 3D topic /openpose_ros/detected_poses_keypoints_3d
  keypoints_3d_pub = nh.advertise<openpose_ros_msgs::PersonDetection_3d>( "/openpose_ros/skeleton_3d/detected_poses_keypoints_3d", 0);


  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // Spin Forever
  ros::spin();

  return 0;
}
