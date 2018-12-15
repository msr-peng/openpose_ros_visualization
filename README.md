OpenPose ROS
============

This work is develped from [here](https://github.com/stevenjj/openpose_ros). I modified most of files to make it support the latest (commit number `5295f2f`) OpenPose library C++ API as well as optimized the code to speed up the wrapper to arrive 10 fps on GTX 1060.

When a depth image is synchronized with the RGB image (RGB-D image), a 3d extractor node has been implemented to obtain 3d pose estimation from the 2d pose estimation gien by OpenPose through the projection of the 2d pose estimation onto the point-cloud of the depth image. Also, a visualization node for the 3d results has been implemented.

# Install OpenPose Library
Install openpose and its dependencies at [here](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md).
  
**NOTE**: Your OpenPose would better build with the same version of OpenCV which comes with ROS, and ROS built-in OpenCV might located in `/opt/ros/kinetic/include`. Otherwise it would cause some problems when operating with image.

**NOTE**: If you choose to build OpenPose by your own custom caffe, you should be careful with your caffe's version. The latest OpenPose Library (commit number `5295f2f`) is only compatible with [caffe](https://github.com/BVLC/caffe) before Aug 14, 2018 (commit number `4353628`).

# Running OpenPose ROS Wrapper and 3d Pose Extractor
**NOTE**: I used ASUS Xtion Pro Live camera, and run it by `openni2_launch` default ROS parameter, so the image resolution is `480*640`. If you get images stream in different resolution, you should modify the following two lines in `skeleton_extract_3d_node.cpp` manually to match you image resolution.
    
![image resolution](https://github.com/msr-peng/openpose_ros/blob/master/images/resolution.png)
    
Now you can run the code. First connect a RGB camera and run the corresponding ROS drivers to start to publish the images (they must be image_raw). For example I used ASUS Xtion Pro Live camera and my command was:
    
   ```bash
   roslaunch openni2_launch openni2.launch
   ```
    
You should get RGB and depth streams in `rqt_image_view`.
    
Then, initialize the 2d detection service node:
    
    ```bash
    rosrun openpose_ros_pkg openpose_ros_node_3d
    ```
    
If everything works fine you should see the following output in your shell:
    
    ```bash
    [INFO] [1501140533.950685432]: Initialization Successful!
    ```
    
Finally, to get the 2d poses of the images from the camera and visualize the output, run:
   ```bash
   rosrun skeleton_extract_3d skeleton_extract_3d_node_test
   ```
Then, images stream with markered skeleton keypoints should be published on topic `/openpose_ros/skeleton_3d/detected_poses_image`, and 3D skeleton keypoints information should published on topic `/openpose_ros/skeleton_3d/detected_poses_keypoints_3d`.
