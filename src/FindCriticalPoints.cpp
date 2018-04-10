#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <highgui.h>

using namespace cv;


Mat src;
Mat dst;
RNG rng(12345);

bool contourTouchesImageBorder(std::vector<Point>& contour, int h, int w)
{
	Rect bb = boundingRect(contour);
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

	//Load the image
	//Mat img = imread( argv[1], 1 );

	//Rotate image 180 degrees
	Mat dst90;
	transpose(img, dst90);
	flip(dst90, dst90, 1);
	Mat dst;
	transpose(dst90, dst);
	flip(dst, dst, 1);

	// Set the lambda matrix the same type and size as input
	lambda = Mat::zeros( dst.rows, dst.cols, dst.type() );

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


/** @function main */
int main( int argc, char** argv )
{
  /// Load an image
  src = PerspectiveTrans(imread( argv[1] ));
  namedWindow( "Source", CV_WINDOW_AUTOSIZE );
  imshow( "Source", src );
  if( !src.data )
  { return -1; }

  //Mat output2;
  	cvtColor(src, src, COLOR_RGB2HSV);
  	inRange(src, Scalar(79, 0, 0), Scalar(139, 255, 255), src);

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
	  if(contourArea(contours[i], false) != 0 && contourTouchesImageBorder(contours[i], src.size().height, src.size().width) == false){
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8,1, 0, Point() );
       circle(src, mc[i],3,Scalar(1,1,1));
       std::cout << "Centroid: ";
       //0.3057 cm/pixel
       //give dimensions based on robot frame - reference point is in bottom left

       std::cout << "x = " << mc[i].x * 0.3057 << " ; y = " << (src.size().height - mc[i].y) * 0.3057 << std::endl;
	  }
     }


  namedWindow( "Source with marker", CV_WINDOW_AUTOSIZE );
  imshow( "Source with marker", src );


  waitKey(0);
  return 0;
  }

