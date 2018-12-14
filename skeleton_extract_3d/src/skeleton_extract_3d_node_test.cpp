// This code synchronizes the 3d point cloud and 2D image and publishes 3d locations of human skeletons
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <openpose_ros_msgs/GetPersons.h>

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

#include <vector>
#include <cmath>

#include <openpose_ros_msgs/BodypartDetection_3d.h>
#include <openpose_ros_msgs/PersonDetection_3d.h>

#include <chrono>


// Set resolution

#define width  640
#define height 480

// Declare Publishers
ros::Publisher           pc_pub;
ros::Publisher           image_pub;   

// Declare 3d keypoints publisher
ros::Publisher	         keypoints_3d_pub;

// Declare rgb camera geometry model
image_geometry::PinholeCameraModel model_;

// Judge whether load rgb camera geometry model
bool CAMERA_MODEL = false;

// Declare rgb camera geometry info subscriber
ros::Subscriber camera_sub;

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
openpose_ros_msgs::BodypartDetection_3d get3dcoordinates(const openpose_ros_msgs::BodypartDetection bodypart_2d, const sensor_msgs::ImageConstPtr& depth_msg, const std::string bodypart_name)
{
  openpose_ros_msgs::BodypartDetection_3d bodypart_depth;

  // Include 2d confidence 3d detections message
  bodypart_depth.confidence = bodypart_2d.confidence;

  // If not detected bodypart
  if (std::isnan(bodypart_depth.confidence) || bodypart_depth.confidence == 0.0 || (bodypart_2d.x == 0 && bodypart_depth.y == 0) || bodypart_2d.x > width || bodypart_2d.y > height )
  {
    notDetectedBodyPart(bodypart_name);
    bodypart_depth.x = NAN;
    bodypart_depth.y = NAN;
    bodypart_depth.z = NAN;
  }

  // If detected bodypart
  else
  {
    ROS_INFO("***********************receive a valid keypoint*****************************");
    // Get keypoint pixel coordinates
    unsigned long long int x_pixel = bodypart_2d.x;
    unsigned long long int y_pixel = bodypart_2d.y;

    std::cout << "x_pixel : " << x_pixel << "\n" << "y_pixel : " << y_pixel << std::endl;

    // Create cv::Point2d object to storage keypoint pixel coordinate
    cv::Point2d keypoint_pixel;
    keypoint_pixel.x = x_pixel;
    keypoint_pixel.y = y_pixel;

    // Create cv::Point3d object to represent 3d ray
    cv::Point3d ray;
    ray = model_.projectPixelTo3dRay(keypoint_pixel);

    std::cout << "ray_x : " << ray.x << "\n" << "ray_y : " << ray.y << "\n" << "ray_z : " << ray.z << std::endl;

    double keypoint_x = 0.0;
    double keypoint_y = 0.0;
    double keypoint_z = 0.0;

    keypoint_z = (depth_msg->data)[y_pixel * width + x_pixel];
    keypoint_x = ray.x * keypoint_z / ray.z;
    keypoint_y = ray.y * keypoint_z / ray.z;

    std::cout << "keypoint_z : " << keypoint_z << "\n" << std::endl;

    bodypart_depth.x = keypoint_x;
    bodypart_depth.y = keypoint_y;
    bodypart_depth.z = keypoint_z;

    std::cout << bodypart_name << " pixel coordinates (x,y,z): " << std::endl;
    std::cout << "( "<< bodypart_2d.x << ", " 
                     << bodypart_2d.y << ", "
                     << bodypart_depth.z << ")" << std::endl;

    std::cout << bodypart_name << " real world coordinates (x,y,z): " << std::endl;
    std::cout << "( " << bodypart_depth.x << ", " 
                      << bodypart_depth.y << ", "
                      << bodypart_depth.z << ")" << std::endl;
  }
  return bodypart_depth;
}

// Declare Service Client
ros::ServiceClient client;
openpose_ros_msgs::GetPersons srv;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

// Declare cameraCallback
void cameraCallback(const sensor_msgs::CameraInfo& camera_info)
{
  model_.fromCameraInfo(camera_info);
  CAMERA_MODEL = true;
  ROS_INFO("Successfully loaded the rgb camera geometry model\n");
  camera_sub.shutdown();
}

// Declare Callback
void callback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& image_msg)
{
  if (CAMERA_MODEL)
  {
  ROS_INFO("Cloud and Image Messages Received!");
  ROS_INFO("    Cloud Time Stamp: %f", depth_msg->header.stamp.toSec());
  ROS_INFO("    Image Time Stamp: %f", image_msg->header.stamp.toSec()); 

  // Publish input pointcloud and image
  pc_pub.publish(*depth_msg);
  image_pub.publish(*image_msg); 
 
  srv.request.image = *image_msg;

  if (client.call(srv))
  {
    // ****************** 4
    auto start = std::chrono::system_clock::now();

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

      // for each body part 
	    for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++)
      {
		    // Initialize bodypart msg 
		    openpose_ros_msgs::BodypartDetection bodypart_detections;
        switch (bodypart_idx)
        {
          case 0: bodypart_detections = skeleton_detections.nose;
                  person_msg.nose = get3dcoordinates(bodypart_detections, depth_msg, "Nose"); break;
          case 1: bodypart_detections = skeleton_detections.neck;
                  person_msg.neck = get3dcoordinates(bodypart_detections, depth_msg, "Neck"); break;
          case 2: bodypart_detections = skeleton_detections.right_shoulder;
                  person_msg.right_shoulder = get3dcoordinates(bodypart_detections, depth_msg, "RShoulder"); break;
          case 3: bodypart_detections = skeleton_detections.right_elbow;
                  person_msg.right_elbow = get3dcoordinates(bodypart_detections, depth_msg, "RElbow"); break;
          case 4: bodypart_detections = skeleton_detections.right_wrist;
                  person_msg.right_wrist = get3dcoordinates(bodypart_detections, depth_msg, "RWrist"); break;
          case 5: bodypart_detections = skeleton_detections.left_shoulder;
                  person_msg.left_shoulder = get3dcoordinates(bodypart_detections, depth_msg, "LShoulder"); break;
          case 6: bodypart_detections = skeleton_detections.left_elbow;
                  person_msg.left_elbow = get3dcoordinates(bodypart_detections, depth_msg, "LElbow"); break;
          case 7: bodypart_detections = skeleton_detections.left_wrist;
                  person_msg.left_wrist = get3dcoordinates(bodypart_detections, depth_msg, "LWrist"); break;
          case 8: bodypart_detections = skeleton_detections.mid_hip;
                  person_msg.mid_hip = get3dcoordinates(bodypart_detections, depth_msg, "MidHip"); break;
          case 9: bodypart_detections = skeleton_detections.right_hip;
                  person_msg.right_hip = get3dcoordinates(bodypart_detections, depth_msg, "RHip"); break;
          case 10:bodypart_detections = skeleton_detections.right_knee;
                  person_msg.right_knee = get3dcoordinates(bodypart_detections, depth_msg, "RKnee"); break;
          case 11:bodypart_detections = skeleton_detections.right_ankle;
                  person_msg.right_ankle = get3dcoordinates(bodypart_detections, depth_msg, "RAnkle"); break;
          case 12:bodypart_detections = skeleton_detections.left_hip;
                  person_msg.left_hip = get3dcoordinates(bodypart_detections, depth_msg, "LHip"); break;
          case 13:bodypart_detections = skeleton_detections.left_knee;
                  person_msg.left_knee = get3dcoordinates(bodypart_detections, depth_msg, "LKnee"); break;
          case 14:bodypart_detections = skeleton_detections.left_ankle;
                  person_msg.left_ankle = get3dcoordinates(bodypart_detections, depth_msg, "LAnkle"); break;
          case 15:bodypart_detections = skeleton_detections.right_eye;
                  person_msg.right_eye = get3dcoordinates(bodypart_detections, depth_msg, "REye"); break;
          case 16:bodypart_detections = skeleton_detections.left_eye;
                  person_msg.left_eye = get3dcoordinates(bodypart_detections, depth_msg, "LEye"); break;
          case 17:bodypart_detections = skeleton_detections.right_ear;
                  person_msg.right_ear = get3dcoordinates(bodypart_detections, depth_msg, "REar"); break;
          case 18:bodypart_detections = skeleton_detections.left_ear;
                  person_msg.left_ear = get3dcoordinates(bodypart_detections, depth_msg, "LEar"); break;
          case 19:bodypart_detections = skeleton_detections.left_big_toe;
                  person_msg.left_big_toe = get3dcoordinates(bodypart_detections, depth_msg, "LBigToe"); break;
          case 20:bodypart_detections = skeleton_detections.left_small_toe;
                  person_msg.left_small_toe = get3dcoordinates(bodypart_detections, depth_msg, "LSmallToe"); break;
          case 21:bodypart_detections = skeleton_detections.left_heel;
                  person_msg.left_heel = get3dcoordinates(bodypart_detections, depth_msg, "LHeel"); break;
          case 22:bodypart_detections = skeleton_detections.right_big_toe;
                  person_msg.right_big_toe = get3dcoordinates(bodypart_detections, depth_msg, "RBigToe"); break;
          case 23:bodypart_detections = skeleton_detections.right_small_toe;
                  person_msg.right_small_toe = get3dcoordinates(bodypart_detections, depth_msg, "RSmallToe"); break;
          case 24:bodypart_detections = skeleton_detections.right_heel;
                  person_msg.right_heel = get3dcoordinates(bodypart_detections, depth_msg, "RHeel"); break;
        }
      }
    }
    // Publish 3D detection
    keypoints_3d_pub.publish(person_msg);

    // ********************** 5
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end-start;
    std::cout << "Time between the gap 4 to 5 " << diff.count() << " s\n";
    start = std::chrono::system_clock::now();
  }
  else
  {
    ROS_INFO("test message");
    ROS_ERROR("Failed to call service detect_poses_3d");
    ROS_ERROR("Did you remap the service and client names?");
    ROS_ERROR("This node expects a service called skeleton_2d_detector in which a launch file should have handled the remapping");
  }
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
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_rect_color", 1);
  client = nh.serviceClient<openpose_ros_msgs::GetPersons>("/detect_poses_3d");

  // Pointcloud publisher topic /openpose_ros/input_pointcloud
  pc_pub = nh.advertise<sensor_msgs::Image>( "/openpose_ros/skeleton_3d/input_depth", 0);

  // Image publisher topic /openpose_ros/input_rgb
  image_pub = nh.advertise<sensor_msgs::Image>( "/openpose_ros/skeleton_3d/input_rgb", 0);

  // Keypoints in 3D topic /openpose_ros/detected_poses_keypoints_3d
  keypoints_3d_pub = nh.advertise<openpose_ros_msgs::PersonDetection_3d>( "/openpose_ros/skeleton_3d/detected_poses_keypoints_3d", 0);


  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // Subscriber to get rgb geometry model
  camera_sub = nh.subscribe("/camera/rgb/camera_info", 1, cameraCallback);

  // Spin Forever
  ros::spin();

  return 0;
}