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

cv::Point2f box_centroid;
cv::Point2f target_centroid;

geometry_msgs::Point box_position_base_frame;
geometry_msgs::Point target_position_base_frame;

cv::Point2f search_centroid_in_area(std::vector<cv::Point2f> centroid_vector, cv::Rect area) {
  float sum_x = 0.0;
  float sum_y = 0.0;
  int number_of_centroids_in_area = 0;
  
  for( int i = 0; i<centroid_vector.size(); i++) {
    if(centroid_vector[i].inside(area)) {
      sum_x += centroid_vector[i].x;
      sum_y += centroid_vector[i].y;
      number_of_centroids_in_area++;
    }
  }
  cv::Point2f extracted_point(sum_x/number_of_centroids_in_area, sum_y/number_of_centroids_in_area);
  return extracted_point;
}

cv::Mat apply_cv_algorithms(cv::Mat camera_image) {
  // convert the image to grayscale format
  cv::Mat img_gray;
  cv::cvtColor(camera_image, img_gray, cv::COLOR_BGR2GRAY);

  cv::Mat canny_output;
  cv::Canny(img_gray,canny_output,10,350);

  return canny_output;
}

std::vector<cv::Point2f> extract_centroids(cv::Mat canny_output) {

  // detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
   std::vector<std::vector<cv::Point>> contours;
   std::vector<cv::Vec4i> hierarchy;
   cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

  // get the moments
  std::vector<cv::Moments> mu(contours.size());
  for( int i = 0; i<contours.size(); i++ )
  { mu[i] = cv::moments( contours[i], false ); }
  
  // get the centroid of figures.
  std::vector<cv::Point2f> centroids(contours.size());
  for( int i = 0; i<contours.size(); i++) {
    float centroid_x = mu[i].m10/mu[i].m00;
    float centroid_y = mu[i].m01/mu[i].m00;
    centroids[i] = cv::Point2f(centroid_x, centroid_y);
  }

    // draw contours
  cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));

  for( int i = 0; i<contours.size(); i++ )
  {
  cv::Scalar color = cv::Scalar(167,151,0); // B G R values
  cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
  cv::circle( drawing, centroids[i], 4, color, -1, 8, 0 );
  }

   // show the resuling image
  cv::namedWindow( "Extracted centroids", cv::WINDOW_AUTOSIZE );
  cv::imshow( "Extracted centroids", drawing );
  cv::waitKey(3);

  return centroids;
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

  cv::Mat canny_output = apply_cv_algorithms(cv_ptr->image);

  std::vector<cv::Point2f> centroids = extract_centroids(canny_output);

  //get box location in 2d image
  cv::Rect box_search_area((image_size_x/2), 0, (image_size_x/2), 255);
  box_centroid = search_centroid_in_area(centroids, box_search_area);

  //get plate location in 2d image
  cv::Rect target_search_area(0, 0, (image_size_x/2), 255);
  target_centroid = search_centroid_in_area(centroids, target_search_area);

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

  geometry_msgs::Point box_position_camera_frame;
  box_position_camera_frame = pixel_to_3d_point(pCloud, box_centroid.x, box_centroid.y);

  geometry_msgs::Point target_position_camera_frame;
  target_position_camera_frame = pixel_to_3d_point(pCloud, target_centroid.x, target_centroid.y);
    
  box_position_base_frame = transform_between_frames(box_position_camera_frame, from_frame, to_frame);
  target_position_base_frame = transform_between_frames(target_position_camera_frame, from_frame, to_frame);

  ROS_INFO_STREAM("3d box position base frame: x " << box_position_base_frame.x << " y " << box_position_base_frame.y << " z " << box_position_base_frame.z);
  ROS_INFO_STREAM("3d target position base frame: x " << target_position_base_frame.x << " y " << target_position_base_frame.y << " z " << target_position_base_frame.z);
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

 

