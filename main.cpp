#include <iostream>
#include <fstream>
//#include <opencv2/cv.h"
//#include "opencv/highgui.h"
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include "circularvoting.h"

using namespace cv;  
using namespace std;

int main()
{
	ifstream input("files.txt");
	for( std::string line; getline( input, line ); )
	{

	Mat img=imread("in/" + line);
	if (!img.data)
	{
		cout << "Image file is invalid!" << endl;
		return -1;
	}
	
	//uchar* p;
	//for(int i = 0; i < nRows; ++i)
	//{
	//	p = img.ptr<uchar>(i);
	//	for (int j = 0; j < nCols; ++j)
	//	{
	//		p[j] = 255;
	//	}
	//}
	
	CircularVoting voting;
	//voting.compute(img);
	Mat out_img(img.rows, img.cols, CV_64FC1, Scalar(0));
	voting.compute(img, out_img,-1);

	
	Mat final(img.rows, img.cols, CV_8UC1, Scalar(0));

	int nRows = final.rows;
	int nCols = final.cols;
	uchar* p;
	double* p1;
	double maxValue = 0;
	for(int i = 0; i < nRows; ++i)
	{
		p1 = out_img.ptr<double>(i);
		for (int j = 0; j < nCols; ++j)
		{
			if (maxValue < p1[j])
			{
				maxValue = p1[j];
			}
		}
	}

	//cout << maxValue << endl;

	for (int R = 3; R < 10; ++R){

		for(int i = 0; i < nRows; ++i)
		{
			p = final.ptr<uchar>(i);
			p1 = out_img.ptr<double>(i);
			for (int j = 0; j < nCols; ++j)
			{
				if (p1[j] > 0.1*R*maxValue)
				{
					p[j] += 1;
				}
			}
		}
	}

	imwrite("out/" + line, final);
	//Mat image(nRows, nCols, CV_16UC1, Scalar(0));
	//Mat abs_img;
	//convertScaleAbs( final, abs_img, 100);//0.0009);
	//imshow("test",abs_img);
	//waitKey(0);
	}
	

	//imshow("Girl", grayimg);
	//imshow("Girl",img); 

	//waitKey();  
	//voting.setParams()


	return 0;
}
