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
  // number of squares in the pattern, a.k.a, interior number of corners
  Size pattern_size(COUNT_SQUARES_X, COUNT_SQUARES_Y);
  // storage for the detected corners in findChessboardConrners
  vector<Point2f> corners;

  bool pattern_found = findChessboardCorners(
            frame, pattern_size, corners,
            CALIB_CB_ADAPTIVE_THRESH +
            CALIB_CB_NORMALIZE_IMAGE +
            CALIB_CB_FAST_CHECK);

  if (!pattern_found)
    return false;

  // if corners are detected, they are further refined by
  // calculating subpixel corners from the grayscale image
  // this iterative process terminates after the given number
  // of iterations and error epsilon
  cornerSubPix(frame, corners, Size(11, 11), Size(-1, -1),
               TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100,
               0.15));
  // draw the detected corners as sanity check
  drawChessboardCorners(frame, pattern_size, Mat(corners),
                        pattern_found);

  // show detected corners in a different window
  imshow("Detected pattern", frame);

  // build a grid of 3D points (z component is 0 because the pattern is
  // in one plane) to fit the square pattern area
  // (COUNT_SQUARES_X * COUNT_SQUARES_Y)


  /*
    ,,Enhance the program, such that multiple camera images are used´´
    => remove the following two lines
  */

  vector<Point3f> pattern_points;
  for (int j = 0; j < COUNT_SQUARES_X * COUNT_SQUARES_Y; ++j)
    pattern_points.push_back(Point3f(j / COUNT_SQUARES_X,
				     j % COUNT_SQUARES_X,
				     0.0f));

  // I do not want to torture my (your) pc, so the number of sets is limited
  if (object_points.size() > maximum_sets ||
      image_points.size() > maximum_sets)
  {
    cout << "Calibration buffer is full. Skipping set...\n";
    return true;
  }

  // populate image points with corners and object points with grid points

  int key_pressed = waitKey(25); // get user key press
  if (key_pressed == 'i') {
    cout << "Saved image for calibration" << endl;
    object_points.push_back(pattern_points);
    image_points.push_back(corners);
  }

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
  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs, tvecs;

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
  bool calibration_ready = false;
  int count_frames = 0;

  for (; ; count_frames++) {
#ifndef QUIET
    cout << "Frame: " << count_frames << endl;
#endif

    capture >> frame; // get a new frame from camera
    Mat gray_frame;
    // convert current frame to grayscale
    cvtColor(frame, gray_frame, CV_BGR2GRAY);
    imshow("Camera Image", frame); // update camera image

    int key_pressed = waitKey(25); // get user key press
    if (key_pressed == 27) // ESC
	    break;

    if (detectPattern(gray_frame, object_points, image_points)) {
	    calibration_ready = true;
	    printf("detected pattern! \n");
	    if (key_pressed == 32) // SPACE
	      break;
    }
  }

  // if at least one video capture contains the pattern, perform calibration
  if (calibration_ready)
  {

    calibrateCamera(object_points, image_points, frame.size(), intrinsic, distCoeffs, rvecs, tvecs);

    cout << "Intrinsic parameters: \n" << intrinsic << endl;
    cout << "distortion params: \n" << distCoeffs << endl;

    for (int i = 0; i < rvecs.size(); i++) {
      cout << "Image " << i << ":" << endl;
      Mat Rotation;
      Rodrigues(rvecs[i], Rotation);
      Vec3d euler =  rmat_to_euler(Rotation);
      cout << "Roll:  " << euler[0] << endl;
      cout << "Pitch: " << euler[1] << endl;
      cout << "Yaw:   " << euler[2] << endl;

      cout << "Translation: " << endl;
      cout << tvecs[i] << endl;

    }
  }
  else
  {
    cerr << "No pattern found in any video capture. Exiting." << endl;
    return -1;
  }

  for (; ; count_frames++) {
#ifndef QUIET
    cout << "Frame: " << count_frames << " (calibrated) " << endl;
#endif
    capture >> frame; // get a new frame from camera

    //cout << "M = " << endl << " "  << intrinsic << endl << endl;
    undistort(frame, undistorted_frame, intrinsic, distCoeffs);
    imshow("Camera Image", undistorted_frame); // update camera image


    int key_pressed = waitKey(25); // get user key press
    if (key_pressed == 's') {
	    imwrite("distorted.png", frame);
	    imwrite("undistorted.png", undistorted_frame);
    }
    if (key_pressed == 27) // ESC
	    break;
  }

  // release camera
  capture.release();

  return 0;
}


int main(int argc, char **argv)
{
  return argc == 1 ? runInteractive(1)
                   : runInteractive(atoi(argv[1]));
}
