//Optical Flow between two images

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{	
	//VideoCapture cap("/home/aayush/Downloads/OpenCV/SnakeOF/Snakes/images/test4.mp4");
	//int frameNum = 1;
	//while(1)
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
		qualityLevel2=0.2;
		qualityLevel1=0.2;
		minDistance1=10;
		minDistance2=10;

		int vert_mag[maxCorners2];
		/*
		cap.set(CV_CAP_PROP_POS_FRAMES,frameNum);
		cap >> im1;

		cap.set(CV_CAP_PROP_POS_FRAMES,frameNum+12);
		cap >> im2;

		im = im1;
		*/
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
		int j =0;
		for(int i =1;i<features1.size();i++)
		{	if(!((features1[i].x>1110)&&(features1[i].y<20)))
			{	if(!((features1[i].x<175)&&(features1[i].y>690)))
				{
					Point p0(ceil(features1[i].x),ceil(features1[i].y));
					Point p1(ceil(features2[i].x),ceil(features2[i].y));
					vert_mag[j] = abs(features2[i].y - features1[i].y);
					arrowedLine(im,p0,p1,CV_RGB(100,100,100),2);
					j++;
				}
			}	
		}

	
		int sizeX = j;
		int sizeY = 100;

		//for(int i =1;i<=sizeX;i++)
		//	if(sizeY<vert_mag[i])
		//		sizeY=vert_mag[i];

	//cout << " Size X "<<sizeX<<endl;
	//cout <<"SizeY "<<sizeY<<endl;

		Mat plot = Mat::zeros(sizeY,sizeX,CV_8UC3);
	
		for (int i=1;i<sizeX;i++)
		{
			Point plotPoint1(i,sizeY);
			Point plotPoint2(i,sizeY-vert_mag[i]);
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

	//	if(waitKey((1000/30)==27))
	//		break;
		waitKey(0);
	//	frameNum+=24;	
	}
	return 0;
}