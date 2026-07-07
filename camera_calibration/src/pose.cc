#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <ostream>

#if CV_VERSION_MAJOR == 4
# include <opencv2/highgui/highgui_c.h> // https://github.com/MIT-SPARK/Kimera-VIO/issues/1
# undef CV_CAP_PROP_FRAME_WIDTH
# undef CV_CAP_PROP_FRAME_HEIGHT
# define CV_CAP_PROP_FRAME_WIDTH CAP_PROP_FRAME_WIDTH
# define CV_CAP_PROP_FRAME_HEIGHT CAP_PROP_FRAME_HEIGHT
#else
# warning "The student assumes OpenCV-Version 4.x.x"
#endif

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std;
using namespace cv;

enum { INTERACTIVE_MODE, PRESPECIFIED_MODE };

#define QUIET

#define KEY_ESCAPE 1048603
#define KEY_SPACE 1048608
#define KEY_CLOSE_WINDOW -1

#define COUNT_SQUARES_X 4
#define COUNT_SQUARES_Y 6
#include <opencv2/opencv.hpp>
#include <cmath>

Vec3d rmat_to_euler(const Mat& R)
{
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +
                          R.at<double>(1,0) * R.at<double>(1,0));

    bool singular = sy < 1e-6;

    double x, y, z;

    if (!singular)
    {
        x = atan2(R.at<double>(2,1), R.at<double>(2,2)); // roll
        y = atan2(-R.at<double>(2,0), sy);               // pitch
        z = atan2(R.at<double>(1,0), R.at<double>(0,0)); // yaw
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    return Vec3d(x, y, z);
}

/**
 * Method for detecting pattern in current frame.
 * Returns world and image coordinates of detected corners via out parameters.
 */
bool detectPattern(Mat frame,
			    vector< vector<Point3f> >& object_points,
			    vector< vector<Point2f> >& image_points,
          int maximum_sets = 50)
{
  Size pattern_size(COUNT_SQUARES_X, COUNT_SQUARES_Y);
  vector<Point2f> corners;

  bool pattern_found = findChessboardCorners(
            frame, pattern_size, corners,
            CALIB_CB_ADAPTIVE_THRESH +
            CALIB_CB_NORMALIZE_IMAGE +
            CALIB_CB_FAST_CHECK);

  if (!pattern_found)
    return false;

  cornerSubPix(frame, corners, Size(11, 11), Size(-1, -1),
               TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100,
               0.15));

  vector<Point3f> pattern_points;
  for (int j = 0; j < COUNT_SQUARES_X * COUNT_SQUARES_Y; ++j)
    pattern_points.push_back(Point3f(j / COUNT_SQUARES_X,
				     j % COUNT_SQUARES_X,
				     0.0f));

  if (object_points.size() > maximum_sets ||
      image_points.size() > maximum_sets)
  {
    cout << "Calibration buffer is full. Skipping set...\n";
    return true;
  }

  object_points.push_back(pattern_points);
  image_points.push_back(corners);

  return true;
}

/**
 * Main method for interactive behavior.
 * Requires user to present calibration pattern in front of camera.
 * By pressing any key an image is grabbed from the camera stream,
 * pressing ESC finishes calibration.
 */
int runInteractive(int iCap = 1)
{
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("pose_publisher");

  auto pose_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/camera_pose", 10);
  auto path_pub =
    node->create_publisher<nav_msgs::msg::Path>(
						"/chessboard_path", 10);

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "camera_frame";

  tf2_ros::TransformBroadcaster tf_broadcaster(node);
  rclcpp::Rate loop_rate(30);

  cout << "Camera calibration using interactive behavior. "
	  << " Press any key to grab frame, ESC to perform calibration.\n";
  // show camera image in a separate window
  namedWindow("Camera Image", CV_WINDOW_KEEPRATIO);

  // current camera frame and first captured frame
  Mat frame, undistorted_frame;
  // storage for object points (world coords)
  // and image points (image coords) for use in calibrateCamera
  vector< vector< Point3f > > object_points;
  vector< vector< Point2f > > image_points;
  // calibration parameters
  float m[3][3] = {{660.0381836895866, 0, 315.952445979791}, {0, 660.1702745351153, 229.9783072067786},{0, 0, 1}};
  Mat intrinsic = Mat(3, 3, CV_32FC1, m);
  Mat distCoeffs = (Mat_<float>(1, 5) << 0.0774595326694079, -0.9822995703352496, 0.001853001111686757, 0.0004869717236538896, 2.010310818542591);
  Mat rvecs, tvecs;

  // open the default camera
  VideoCapture capture(iCap);

  // check if opening camera stream succeeded
  if (!capture.isOpened())
  {
    cerr << "Camera could not be opened. Try another index as argument? Exiting.\n";
    return -1;
  }

  // set frame width and height by hand, defaults to 160x120
  capture.set (CV_CAP_PROP_FRAME_WIDTH, 640);
  capture.set (CV_CAP_PROP_FRAME_HEIGHT, 480);

  // flag for determining whether pattern was detected in at
  // least one of the camera grabs
  int count_frames = 0;

  Mat Rotation;
  for (;; count_frames++) {
    capture >> frame; // get a new frame from camera

    undistort(frame, undistorted_frame, intrinsic, distCoeffs);

    Mat gray_frame;
    cvtColor(undistorted_frame, gray_frame, CV_BGR2GRAY);
    if (detectPattern(gray_frame, object_points, image_points)) {

      bool result = solvePnP(object_points.back(), image_points.back(),
                             intrinsic, distCoeffs, rvecs, tvecs);
      if (result) {

        Rodrigues(rvecs, Rotation);

        Vec3d euler = rmat_to_euler(Rotation);
        char text[200];

        snprintf(
            text, sizeof(text),
            "roll: %.2f  pitch: %.2f  yaw: %.2f, x: %.2f, y: %.2f, z: %.2f",
            euler[0], euler[1], euler[2], (float)tvecs.at<double>(0),
            (float)tvecs.at<double>(1), (float)tvecs.at<double>(2));
        putText(undistorted_frame, text, Point(10, 20), FONT_HERSHEY_DUPLEX,
                0.5, CV_RGB(255, 255, 255), 1);

        geometry_msgs::msg::PoseStamped pose_msg;

        pose_msg.header.stamp = node->now();
        pose_msg.header.frame_id = "camera_frame";

        pose_msg.pose.position.x = tvecs.at<double>(0);
        pose_msg.pose.position.y = tvecs.at<double>(1);
        pose_msg.pose.position.z = tvecs.at<double>(2);

        Mat R;
        Rodrigues(rvecs, R);

        tf2::Matrix3x3 tf_rot(
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

        tf2::Quaternion q;
        tf_rot.getRotation(q);

        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_pub->publish(pose_msg);
	cout << "publishing pose_msg" << endl;

	pose_msg.header.frame_id = "camera_frame";

	path_msg.header.stamp = node->now();
	path_msg.poses.push_back(pose_msg);

	if (path_msg.poses.size() > 50)
	  {
	    path_msg.poses.erase(path_msg.poses.begin());
	  }

	path_pub->publish(path_msg);




	geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = node->now();
        tf_msg.header.frame_id = "camera_frame";
        tf_msg.child_frame_id = "chessboard";

        tf_msg.transform.translation.x = tvecs.at<double>(0);
        tf_msg.transform.translation.y = tvecs.at<double>(1);
        tf_msg.transform.translation.z = tvecs.at<double>(2);

        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster.sendTransform(tf_msg);
      }

      object_points.clear();
      image_points.clear();
    }

    imshow("Camera Image", undistorted_frame); // update camera image


    int key_pressed = waitKey(25); // get user key press
    if (key_pressed == 27) // ESC
	    break;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  // release camera
  rclcpp::shutdown();
  capture.release();

  return 0;
}

int main(int argc, char **argv)
{
  return argc == 1 ? runInteractive(1)
                   : runInteractive(atoi(argv[1]));
}
