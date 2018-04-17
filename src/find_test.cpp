#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

RNG rng = rng(12345);

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  double x, y;
  Mat src;
  Mat dst;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  Mat PerspectiveTrans( Mat img )
  {
    // Input Quadilateral or Image plane coordinates
    Point2f inputMat[4];
    // Output Quadilateral or World plane coordinates
    Point2f outputMat[4];

    // Lambda Matrix
    Mat lambda( 2, 4, CV_32FC1 );
    //Input and Output Image;
    Mat input, output;

    //Rotate image 180 degrees
    Mat dst90;
    transpose(img, dst90);
    flip(dst90, dst90, 1);
    Mat dst;
    transpose(dst90, dst);
    flip(dst, dst, 1);

    // Set the lambda matrix the same type and size as input
    lambda = Mat::zeros( dst.rows, dst.cols, dst.type() );

    // Set Perspective Transform matrix points from calibration image
    inputMat[0] = Point2f( 125, 477);
    inputMat[1] = Point2f( 463, 480);
    inputMat[2] = Point2f( 409, 191);
    inputMat[3] = Point2f( 191, 195);

    outputMat[0] = Point2f( 200, 480);
    outputMat[1] = Point2f( 420, 480);
    outputMat[2] = Point2f( 420, 260);
    outputMat[3] = Point2f( 200, 260);

    // Get the Perspective Transform Matrix i.e. lambda
    lambda = getPerspectiveTransform( inputMat, outputMat );
    // Apply the Perspective Transform just found to the src image
    warpPerspective(dst,output,lambda,dst.size() );

    waitKey(0);
    return output;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    src = PerspectiveTrans(cv_ptr->image);

  

    //convert to HSV colorspace
    /*cvtColor(src, src, COLOR_RGB2HSV);
    //find green parts of image
    inRange(src, Scalar(38, 160, 60), Scalar(75, 255, 255), src);
  
    std::vector<std::vector<Point> > contours;
    Mat edges = src;
    findContours(edges, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    //find moments of each contour
    std::vector<Moments> mu(contours.size());
    for( int i = 0; i< contours.size(); i++)
    {mu[i] = moments(contours[i], false);}

    //find center point of each contour
    std::vector<Point> mc(contours.size());
    for( int i = 0; i< contours.size(); i++)
    {mc[i] = Point(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);}

    /// Draw contours
    Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
       {
      if(contourArea(contours[i], false) != 0){
         //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         //drawContours( drawing, contours, i, color, 2, 8,1, 0, Point() );
         //circle(src, mc[i],3,Scalar(1,1,1));

         //0.3057 cm/pixel
         //give dimensions based on robot frame - reference point is in bottom left
         mc[i].y = src.size().height - mc[i].y;
         x = mc[i].y * 0.393 + 23;
         //determine y value considering black corners of transformed image
         if (mc[i].y < 263)
           y = (296 - mc[i].x) * 0.393;
         else
           y = (296 -(mc[i].x - 115 + mc[i].y*(114/21))) * 0.393;

         //std::cout << "x = " << x << " ; y = " <<  y << std::endl;
     }
      }
    */

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, src);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_test");
  ImageConverter ic;
  ros::spin();
  return 0;
}