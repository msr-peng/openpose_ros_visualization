// OpenPose dependencies
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/pose/poseParameters.hpp>
#include <openpose/utilities/headers.hpp>

// 3rdparty dependencies
#include <std_srvs/Empty.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/init.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <openpose_ros_msgs/GetPersons.h>

#include <iostream>
#include <chrono>

// GFlags: DEFINE_bool, _int32, _int64, _uint64, _double, _string
#include <gflags/gflags.h>
// Allow Google Flags in Ubuntu 14
#ifndef GFLAGS_GFLAGS_H_
    namespace gflags = google;
#endif

DEFINE_int32(logging_level,             3,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
                                                        " 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
                                                        " low priority messages and 4 for important ones.");
// Producer
DEFINE_string(image_path,               "/root/catkin_ws/COCO_val2014_000000000192.jpg",     "Process the desired image.");
// OpenPose
DEFINE_string(model_pose,               "BODY_25",      "Model to be used. E.g., `COCO` (18 keypoints), `MPI` (15 keypoints, ~10% faster), "
                                                        "`MPI_4_layers` (15 keypoints, even faster but less accurate).");
DEFINE_string(model_folder,             "/root/catkin_ws/src/openpose_ros_pkg/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
DEFINE_string(net_resolution,           "-1x368",       "Multiples of 16. If it is increased, the accuracy potentially increases. If it is"
                                                        " decreased, the speed increases. For maximum speed-accuracy balance, it should keep the"
                                                        " closest aspect ratio possible to the images or videos to be processed. Using `-1` in"
                                                        " any of the dimensions, OP will choose the optimal aspect ratio depending on the user's"
                                                        " input value. E.g., the default `-1x368` is equivalent to `656x368` in 16:9 resolutions,"
                                                        " e.g., full HD (1980x1080) and HD (1280x720) resolutions.");
DEFINE_string(output_resolution,        "-1x-1",        "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " input image resolution.");
DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
DEFINE_double(scale_gap,                0.3,              "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");
// OpenPose Rendering
DEFINE_bool(disable_blending,           false,          "If enabled, it will render the results (keypoint skeletons or heatmaps) on a black"
                                                        " background, instead of being rendered into the original image. Related: `part_to_show`,"
                                                        " `alpha_pose`, and `alpha_pose`.");
DEFINE_double(render_threshold,         0.05,           "Only estimated keypoints whose score confidences are higher than this threshold will be"
                                                        " rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;"
                                                        " while small thresholds (~0.1) will also output guessed and occluded keypoints, but also"
                                                        " more false positives (i.e., wrong detections).");
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
                                                        " hide it. Only valid for GPU rendering.");

std::map<unsigned int, std::string> bodypart_map;
op::Point<int> netInputSize;
op::Point<int> outputSize;
op::PoseModel poseModel;
std::shared_ptr<op::ScaleAndSizeExtractor> p_scale_size_extractor;
std::shared_ptr<op::PoseExtractorCaffe> p_pose_extractor;
std::shared_ptr<op::PoseCpuRenderer> p_pose_renderer;

ros::Publisher           keypoints_pub;
ros::Publisher           image_skeleton_pub;

openpose_ros_msgs::BodypartDetection getBodyPartDetectionFromArrayAndIndex(const op::Array<float>& array, size_t idx)
{
  openpose_ros_msgs::BodypartDetection bodypart;

  bodypart.x = array[idx];
  bodypart.y = array[idx+1];
  bodypart.confidence = array[idx+2];
  return bodypart;
}

openpose_ros_msgs::BodypartDetection getNANBodypart()
{
  openpose_ros_msgs::BodypartDetection bodypart;
  bodypart.confidence = NAN;
  return bodypart;
}

bool detectPosesCallback(openpose_ros_msgs::GetPersons::Request& req, openpose_ros_msgs::GetPersons::Response& res)
{
  ROS_INFO("detectPosesCallback");

  // **************** 1
  auto start = std::chrono::system_clock::now();

  // Initialize output
  op::CvMatToOpInput cvMatToOpInput{op::PoseModel::BODY_25};
  op::CvMatToOpOutput cvMatToOpOutput;
  op::OpOutputToCvMat opOutputToCvMat;
  op::FrameDisplayer frameDisplayer{"OpenPose Example", outputSize};

  // Step 1 - Read image, error if empty
  // Convert ROS message to opencv image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(req.image, req.image.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("detectPosesCallback cv_bridge exception: %s", e.what());
    return false;
  }
  cv::Mat inputImage = cv_ptr->image;
  // cv::Mat inputImage = op::loadImage(FLAGS_image_path, CV_LOAD_IMAGE_COLOR);
  if(inputImage.empty())
    op::error("Could not open or find the image: " + FLAGS_image_path, __LINE__, __FUNCTION__, __FILE__);
  const op::Point<int> imageSize{inputImage.cols, inputImage.rows};

  // Step 2 - Get desired scale sizes
  std::vector<double> scaleInputToNetInputs;
  std::vector<op::Point<int>> netInputSizes;

  double scaleInputToOutput;
  op::Point<int> outputResolution;
  std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
      = p_scale_size_extractor->extract(imageSize);

  // Step 3 - Format input image to OpenPose input and output formats
  const auto netInputArray = cvMatToOpInput.createArray(inputImage, scaleInputToNetInputs, netInputSizes);
  auto outputArray = cvMatToOpOutput.createArray(inputImage, scaleInputToOutput, outputResolution);
  
  // *********************** 2
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end-start;
  std::cout << "Time between the gap 1 to 2 " << diff.count() << " s\n";
  start = std::chrono::system_clock::now();

  // Step 4 - Estimate poseKeypoints
  p_pose_extractor->forwardPass(netInputArray, imageSize, scaleInputToNetInputs);

  // ********************** 3
  end = std::chrono::system_clock::now();
  diff = end-start;
  std::cout << "Time between the gap 2 to 3 " << diff.count() << " s\n";
  start = std::chrono::system_clock::now();

  const auto poseKeypoints = p_pose_extractor->getPoseKeypoints();
  
  // Step 5 - Render poseKeypoints
  p_pose_renderer->renderPose(outputArray, poseKeypoints, scaleInputToOutput);
  
  // Step 6 - OpenPose output format to cv::Mat
  auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);


  sensor_msgs::Image ros_image;
  cv_bridge::CvImagePtr cv_ptr_out = cv_bridge::toCvCopy(req.image, req.image.encoding);
  cv_ptr_out->image = outputImage;
  ros_image = *(cv_ptr_out->toImageMsg());
  //frameDisplayer.displayFrame(outputImage, 0); // Alternative: cv::imshow(outputImage) + cv::waitKey(0)
  image_skeleton_pub.publish(ros_image);
  // End Visualize Output


  if (!poseKeypoints.empty() && poseKeypoints.getNumberDimensions() != 3)
  {
    ROS_ERROR("pose.getNumberDimensions(): %d != 3", (int) poseKeypoints.getNumberDimensions());
    return false;
  }

  int num_people = poseKeypoints.getSize(0);
  int num_bodyparts = 25;
  // int num_bodyparts = (poseKeypoints.getSize(1) - 1);

  ROS_INFO("num people: %d", num_people);

  for (size_t person_idx = 0; person_idx < num_people; person_idx++)
  {
    ROS_INFO("    Person ID: %zu", person_idx);

    openpose_ros_msgs::PersonDetection person_msg;

    //add number of people detected
    person_msg.num_people_detected = num_people;

    //add person ID
    person_msg.person_ID = person_idx;	

    // Initialize all bodyparts with nan
    // (commented this line) --> openpose_ros_msgs::PersonDetection person_msg;
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

    for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++)
    {
      size_t final_idx = 3*(person_idx*num_bodyparts + bodypart_idx);

      const std::string body_part_string = bodypart_map[bodypart_idx];

      openpose_ros_msgs::BodypartDetection bodypart_detection = getBodyPartDetectionFromArrayAndIndex(poseKeypoints, final_idx);

      if (body_part_string == "Nose") person_msg.nose = bodypart_detection;
      else if (body_part_string == "Neck") person_msg.neck = bodypart_detection;
      else if (body_part_string == "RShoulder") person_msg.right_shoulder = bodypart_detection;
      else if (body_part_string == "RElbow") person_msg.right_elbow = bodypart_detection;
      else if (body_part_string == "RWrist") person_msg.right_wrist = bodypart_detection;
      else if (body_part_string == "LShoulder") person_msg.left_shoulder = bodypart_detection;
      else if (body_part_string == "LElbow") person_msg.left_elbow = bodypart_detection;
      else if (body_part_string == "LWrist") person_msg.left_wrist = bodypart_detection;
      else if (body_part_string == "MidHip") person_msg.mid_hip = bodypart_detection;
      else if (body_part_string == "RHip") person_msg.right_hip = bodypart_detection;
      else if (body_part_string == "RKnee") person_msg.right_knee = bodypart_detection;
      else if (body_part_string == "RAnkle") person_msg.right_ankle = bodypart_detection;
      else if (body_part_string == "LHip") person_msg.left_hip = bodypart_detection;
      else if (body_part_string == "LKnee") person_msg.left_knee = bodypart_detection;
      else if (body_part_string == "LAnkle") person_msg.left_ankle = bodypart_detection;
      else if (body_part_string == "REye") person_msg.right_eye = bodypart_detection;
      else if (body_part_string == "LEye") person_msg.left_eye = bodypart_detection;
      else if (body_part_string == "REar") person_msg.right_ear = bodypart_detection;
      else if (body_part_string == "LEar") person_msg.left_ear = bodypart_detection;
      else if (body_part_string == "LBigToe") person_msg.left_big_toe = bodypart_detection;
      else if (body_part_string == "LSmallToe") person_msg.left_small_toe = bodypart_detection;
      else if (body_part_string == "LHeel") person_msg.left_heel = bodypart_detection;
      else if (body_part_string == "RBigToe") person_msg.right_big_toe = bodypart_detection;
      else if (body_part_string == "RSmallToe") person_msg.right_small_toe = bodypart_detection;
      else if (body_part_string == "RHeel") person_msg.right_heel = bodypart_detection;
      else
      {
        ROS_INFO("Unknown bodypart %s", body_part_string.c_str());
      }
      ROS_INFO("        body part: %s", body_part_string.c_str());
      ROS_INFO("            (x, y, confidence): %i, %i, %f", bodypart_detection.x, bodypart_detection.y, bodypart_detection.confidence);
    }
    //publish keypoints data of person_msg	
    keypoints_pub.publish(person_msg);
    res.detections.push_back(person_msg);
  }
  ROS_INFO("Detected %d persons", (int) res.detections.size());

  // ********************** 4
  end = std::chrono::system_clock::now();
  diff = end-start;
  std::cout << "Time between the gap 3 to 4 " << diff.count() << " s\n";
  start = std::chrono::system_clock::now();

  return true;
}

int main(int argc, char** argv)
{
  // Parsing command line flags
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "openpose_ros_service_node_3d");  

  ros::NodeHandle local_nh("~");

  // Read GFlags (user defined configuration)
  // outputSize
  outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
  // netInputSize
  netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
  // poseModel
  poseModel = op::flagsToPoseModel(FLAGS_model_pose);
  // bodyMap
  bodypart_map = getPoseBodyPartMapping(poseModel);

  // Initialize all required classes
  p_scale_size_extractor = std::shared_ptr<op::ScaleAndSizeExtractor>(
    new op::ScaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap));
  
  p_pose_extractor = std::shared_ptr<op::PoseExtractorCaffe>(
    new op::PoseExtractorCaffe{poseModel, FLAGS_model_folder, FLAGS_num_gpu_start});
  p_pose_renderer = std::shared_ptr<op::PoseCpuRenderer>(
    new op::PoseCpuRenderer{poseModel, (float)FLAGS_render_threshold, !FLAGS_disable_blending,
                                        (float)FLAGS_alpha_pose});

  ros::NodeHandle nh;
  image_skeleton_pub = nh.advertise<sensor_msgs::Image>( "/openpose_ros/skeleton_3d/detected_poses_image", 0 );  
  //declare publisher of type openpose_ros_msgs::PersonDetection in topic /openpose_ros/detected_poses_keypoints
  keypoints_pub = nh.advertise<openpose_ros_msgs::PersonDetection>( "/openpose_ros/skeleton_3d/detected_poses_keypoints" , 0 );

  ros::ServiceServer service = nh.advertiseService("detect_poses_3d", detectPosesCallback);

  p_pose_extractor->initializationOnThread();
  p_pose_renderer->initializationOnThread();

  ROS_INFO("Initialization Successful!");
  ros::spin();

  return 0;
}