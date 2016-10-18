//Optical Flow between two images


#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	Mat im1,im2,im;
	vector<Point2f> features1,features2;
	vector<uchar> status,features_found;
	vector<float> err;
	int maxCorners1,maxCorners2;
	double qualityLevel1,qualityLevel2;
	double minDistance1,minDistance2;

	maxCorners2=100;
	maxCorners1=100;
	qualityLevel2=0.5;
	qualityLevel1=0.5;
	minDistance1=.20;
	minDistance2=.20;

	int vert_mag[maxCorners2];


	im1 = imread(argv[1],0);
	im2 = imread(argv[2],0);
	im = imread(argv[1],1);


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
	//features_found.reserve(maxCorners2);
	//cout << features_found.size();
	//cout << nxtpts;

	for(int i =1;i<maxCorners2;i++)
	{	
		Point p0(ceil(features1[i].x),ceil(features1[i].y));
		Point p1(ceil(features2[i].x),ceil(features2[i].y));
		vert_mag[i] = abs(features2[i].y - features1[i].y);
		arrowedLine(im,p0,p1,CV_RGB(100,100,100),2);

	}

	int sizeX = features1.size();
	int sizeY = 0;

	for(int i =1;i<=sizeX;i++)
		if(sizeY<vert_mag[i])
			sizeY=vert_mag[i];

	Mat plot = Mat::ones(sizeX,sizeY,CV_8UC3);

	for (int i=1;i<sizeX;i++)
	{
		Point plotPoint1(i,maxCorners1);
		Point plotPoint2(i,vert_mag[i]);
		line(plot,plotPoint1,plotPoint2,CV_RGB(255,255,255));
	}
	
	for(int i=1;i<sizeX;i++)
		cout<<vert_mag[i]<<endl;

	namedWindow("Image",0);
	imshow("Image",plot);

	namedWindow("Image2",0);
	imshow("Image2",im);

	/*cout<<"Features1.x= "<<features1[1].x<<endl;
	cout<<"Features1.y = "<<features1[1].y<<endl;
	cout<<"Features2.x= "<<features2[1].x<<endl;
	cout<<"Features2.y= "<<features2[1].y<<endl;
	*/

	waitKey(0);
	return 0;
}


