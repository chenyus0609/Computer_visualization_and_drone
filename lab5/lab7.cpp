#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <math.h>
#include <string.h>



using namespace cv;
using namespace std;


void distance_detection(Rect rect) {
	
}

static double angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1 * dy1)*(dx2*dx2 + dy2 * dy2) + 1e-10);
}

int main(int argc, char const *argv[])
{
	//get frames
	VideoCapture cap(0); // 0: default device
	//CascadeClassifier宣告
	String face_cascade_name = "haarcascade_frontalface_default.xml";
	CascadeClassifier face_cascade;// , face_cascade2, face_cascade3;
	face_cascade.load(face_cascade_name);
	while (1)
	{
		Mat frame, grayframe;
		cap >> frame;
		cvtColor(frame, grayframe, CV_BGR2GRAY);
		blur(grayframe, grayframe, Size(3, 3));

		//findContour所需變數
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		int mode = CV_RETR_LIST;
		int method = CV_CHAIN_APPROX_SIMPLE;
		Point offset = Point(0, 0);
		/// ??像?行二值化
		Mat threshold_output;
		threshold(grayframe, threshold_output, 100, 255, THRESH_BINARY);
		findContours(threshold_output, contours, hierarchy, mode, method, offset);

		vector <Point> poly(contours.size());
		vector<vector<Point> > squares;
		for (int i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), poly, 15, true); //15可換成arcLength(Mat(contours[i]), true)*0.02
			//15越小越曲
			//approxpolydp到這邊就完了


			// square contours should have 4 vertices after approximation
			// relatively large area (to filter out noisy contours)
			// and be convex.
			// Note: absolute value of an area is used because
			// area may be positive or negative - in accordance with the
			// contour orientation
			if (poly.size() == 4 &&
				fabs(contourArea(Mat(poly))) > 1000 &&
				isContourConvex(Mat(poly)))
			{
				double maxCosine = 0;

				for (int j = 2; j < 5; j++)
				{
					// find the maximum cosine of the angle between joint edges
					double cosine = fabs(angle(poly[j % 4], poly[j - 2], poly[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}

				// if cosines of all angles are small
				// (all angles are ~90 degree) then write quandrange
				// vertices to resultant sequence
				if (maxCosine < 0.7)
					squares.push_back(poly);
				//2是線的粗度 0 255 255是黃色
			}
		}

		
		
		//face dection
		
		
		vector<Rect> rect;
		face_cascade.detectMultiScale(grayframe, rect, 1.1, 3, 0);
		cout << "rect_size: " << rect.size() << "\n";
		//face_cascade2.detectMultiScale(grayframe, rect, 1.1, 3, 0);
		//face_cascade3.detectMultiScale(grayframe, rect, 1.1, 3, 0);
		for (size_t i = 0; i < squares.size(); i++)
		{
			const Point p = squares[i][0];
			//cout << "square_size: " << squares.size() << "\n";
			int n = (int)squares[i].size();
			//don't detect the border
			if (p.x > 3 && p.y > 3)
			{
				drawContours(frame, squares, i, Scalar(0, 255, 255), 3, 8);
				
				for (int j = 0; j < rect.size(); j++)
				{
					Point  center;
					int radius;
					center.x = cvRound((rect[j].x + rect[j].width * 0.5));
					center.y = cvRound((rect[j].y + rect[j].height * 0.5));
					cout << "point1: " << squares[i][0] << " point2: " << squares[i][1] << " point3: " << squares[i][2] << " point4: " << squares[i][3] << "\n";
					cout << "center_x: " << center.x << " center_y: " << center.y << endl;
					int min[2] = { 10000, 10000 }, max[2] = { 0, 0 };
					for (int k = 0; k < 4; k++) {
						if (squares[i][k].x < min[0])
							min[0] = squares[i][0].x;
						if (squares[i][k].y < min[1])
							min[1] = squares[i][k].y;
						if (squares[i][k].x > max[0])
							max[0] = squares[i][k].x;
						if (squares[i][k].x > max[1])
							max[1] = squares[i][k].y;
					}
					if (center.x > min[0] && center.x < max[0] && center.y > min[1] && center.y < max[1])
					{
						radius = cvRound((rect[j].width + rect[j].height) * 0.25);
						circle(frame, center, radius, CV_RGB(255, 0, 0), 2);
						cout << "Area: " << rect[j].area() << endl;
					}
					
				}
			}

			//polylines(frame, p, n, 1, true, Scalar(0, 255, 0), 3, LINE_AA);
		}
		
		

		namedWindow("webcam", WINDOW_AUTOSIZE);
		imshow("webcam", frame);
		waitKey(33);


	}

	return 0;

}