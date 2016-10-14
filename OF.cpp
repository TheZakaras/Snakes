#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	Mat im1,im2;
	//IplImage* im1 = 0;
	//IplImage* im2 = 0; 
	vector<Point2f> features1,features2;
	vector<uchar> status;
	vector<float> err;
	int maxCorners1,maxCorners2;
	double qualityLevel1,qualityLevel2;
	double minDistance1,minDistance2;

	maxCorners2=500;
	maxCorners1=500;
	qualityLevel2=0.01;
	qualityLevel1=0.01;
	minDistance1=20.;
	minDistance2=20.;

	im1 = imread(argv[1],0);
	im2 = imread(argv[2],0);
	//im1 = cvLoadImage("scene00181.png",
	//	CV_LOAD_IMAGE_GRAYSCALE);

	//im2 = cvLoadImage("scene00193.png",
	//	CV_LOAD_IMAGE_GRAYSCALE);

	//im11.convertTo(im1,CV_32FC1);
	//im21.convertTo(im2,CV_32FC1);
	//cvtColor(im11,)
	
	//namedWindow("image",WINDOW_AUTOSIZE);
	//imshow("Image",im11);



	goodFeaturesToTrack(im1,features1,maxCorners1,qualityLevel1,minDistance1,Mat(),3,0,0.04);

	goodFeaturesToTrack(im2,features2,maxCorners2,qualityLevel2,minDistance2,Mat(),3,0,0.04);

	/*for (size_t i =0;i<features1.size();i++)
		cv::circle(im1,features1[i],10,cv::Scalar(255.),-1);

	for (size_t i =0;i<features2.size();i++)
		cv::circle(im2,features2[i],10,cv::Scalar(255.),-1);	

	namedWindow("image1",WINDOW_AUTOSIZE);
	imshow(argv[1],im1);

	namedWindow("image2",WINDOW_AUTOSIZE);
	imshow(argv[2],im2);
	*/
	//features2 = features1;
	calcOpticalFlowPyrLK(im1,im2,features1,features2,status,err);

	for (size_t i =0;i<features2.size();i++)
		cv::circle(im2,features2[i],3,Scalar(0,255,0),-1,0);

	namedWindow("image2",WINDOW_AUTOSIZE);
	imshow(argv[2],im2);	

	for (size_t i =0;i<features1.size();i++)
		cv::circle(im1,features1[i],3,cv::Scalar(255.),-1);

	namedWindow("image1",WINDOW_AUTOSIZE);
	imshow(argv[1],im1);


	waitKey(0);
	return 0;
}