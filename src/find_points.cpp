//get images from topic "simple_camera/image_raw", 

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>

static const std::string OPENCV_WINDOW = "Open-CV display window";

using namespace std;

cv::Mat src;
cv::Mat dst;
cv::RNG rng(12345);

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:

    ImageConverter(ros::NodeHandle &nodehandle)
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("simple_camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

		

        
        // Update GUI Window; this will display processed images on the open-cv viewer.
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Also, publish the processed image as a ROS message on a ROS topic
        // can view this stream in ROS with: 
        //rosrun imagview image_view image:=/image_converter/output_video
        image_pub_.publish(cv_ptr->toImageMsg());

    }
}; //end of class definition
    
    bool contourTouchesImageBorder(std::vector<cv::Point>& contour, int h, int w)
	{
		cv::Rect bb = cv::boundingRect(contour);
		bool retval = false;

		int xMin, xMax, yMin, yMax;

		xMin = 0; xMax = w;
		yMin = 0; yMax = h;

		if( bb.x <= xMin ||
			bb.y <= yMin ||
			bb.x + bb.width >= xMax ||
			bb.y + bb.height >= yMax)
		{retval = true;}

		return retval;
	}
	
	cv::Mat PerspectiveTrans( cv::Mat img )
	{
		// Input Quadilateral or Image plane coordinates
		cv::Point2f inputMat[4];
		// Output Quadilateral or World plane coordinates
		cv::Point2f outputMat[4];

		// Lambda Matrix
		cv::Mat lambda( 2, 4, CV_32FC1 );
		//Input and Output Image;
		cv::Mat input, output;

		//Load the image
		//Mat img = imread( argv[1], 1 );

		//Rotate image 180 degrees
		cv::Mat dst90;
		cv::transpose(img, dst90);
		cv::flip(dst90, dst90, 1);
		cv::Mat dst;
		cv::transpose(dst90, dst);
		cv::flip(dst, dst, 1);

		// Set the lambda matrix the same type and size as input
		lambda = cv::Mat::zeros( dst.rows, dst.cols, dst.type() );

		inputMat[0] = cv::Point2f( 125, 477);
		inputMat[1] = cv::Point2f( 463, 480);
		inputMat[2] = cv::Point2f( 409, 191);
		inputMat[3] = cv::Point2f( 191, 195);

		outputMat[0] = cv::Point2f( 200, 480);
		outputMat[1] = cv::Point2f( 420, 480);
		outputMat[2] = cv::Point2f( 420, 260);
		outputMat[3] = cv::Point2f( 200, 260);

		// Get the Perspective Transform Matrix i.e. lambda
		lambda = cv::getPerspectiveTransform( inputMat, outputMat );
		// Apply the Perspective Transform just found to the src image
		cv::warpPerspective(dst,output,lambda,dst.size() );


		cv::waitKey(0);
		return output;
	}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_points");
    ros::NodeHandle n; //        
    ImageConverter ic(n);
    
    ros::Duration timer(0.1);
    double x, y, z;
    while (ros::ok()) {
        /// Load an image
		  src = PerspectiveTrans(cv::imread(argv[1]));
		  cv::namedWindow( "Source", CV_WINDOW_AUTOSIZE );
		  cv::imshow( "Source", src );
		  if( !src.data )
		  { return -1; }

		  //Mat output2;
		  	cv::cvtColor(src, src, cv::COLOR_RGB2HSV);
		  	cv::inRange(src, cv::Scalar(79, 0, 0), cv::Scalar(139, 255, 255), src);

		  std::vector<std::vector<cv::Point> > contours;
		  cv::Mat edges = src;
		  cv::findContours(edges, contours, V_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);


		  //find moments of each contour
		  std::vector<cv::Moments> mu(contours.size());
		  for( int i = 0; i< contours.size(); i++)
		  {mu[i] = cv::moments(contours[i], false);}

		  //find center point of each contour
		  std::vector<cv::Point> mc(contours.size());
		  for( int i = 0; i< contours.size(); i++)
		  {mc[i] = cv::Point(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);}


		  /// Draw contours
		  cv::Mat drawing = cv::Mat::zeros( edges.size(), CV_8UC3 );
		  for( int i = 0; i< contours.size(); i++ )
		     {
			  if(cv::contourArea(contours[i], false) != 0 && contourTouchesImageBorder(contours[i], src.size().height, src.size().width) == false){
		       cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		       cv::drawContours( drawing, contours, i, color, 2, 8,1, 0, cv::Point() );
		       cv::circle(src, mc[i],3,cv::Scalar(1,1,1));
		       std::cout << "Centroid: ";
		       //0.3057 cm/pixel
		       //give dimensions based on robot frame - reference point is in bottom left

		       std::cout << "x = " << mc[i].x * 0.3057 << " ; y = " << (src.size().height - mc[i].y) * 0.3057 << std::endl;
			  }
		     }


		  cv::namedWindow( "Source with marker", CV_WINDOW_AUTOSIZE );
		  cv::imshow( "Source with marker", src );


 	 		cv::waitKey(0);

        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
