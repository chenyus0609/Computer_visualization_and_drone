#include "ardrone/ardrone.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
using namespace cv;
using namespace std;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    // AR.Drone class
    ARDrone ardrone;

	//calibrate
	int i = 15;  //capture 15 frames
	Mat frame;
	Mat out;
	Mat intrinsic;
	Mat distortionCoeffs;
	Mat outputMapX;
	Mat outputMapY;
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	vector<Point2f> frame_corners;
	vector<vector<Point2f> > corners_2d;
	vector<vector<Point3f> > points_3d;
	Size imgsize(7, 5);

	Mat image;
    // Initialize
    /*if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'C'     -- Change camera         *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************" << std::endl;*/

	//dictionary
	//cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	const float markerLength = 12.7;
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	
	VideoCapture cap(1);
	//calibrate camera
	while (i) {
		// Get an image
		//frame = ardrone.getImage();
		cap >> frame;
		cvtColor(frame, frame, CV_RGB2GRAY);
		bool n = findChessboardCorners(frame, Size(7, 5), frame_corners);
		if (!n) continue; //to make sure we capture the chessboard. If not, capture again
		cornerSubPix(frame, frame_corners, Size(15, 15), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.1));
		cout << i << "\n";

		vector<Point3f> temp_3d;
		//corners_2d.insert(corners_2d.end(), frame_corners.begin(), frame_corners.end());
		corners_2d.push_back(frame_corners);
		for (size_t i = 0; i < 5; i++) {
			for (size_t j = 0; j < 7; j++) {
				Point3f curr_point;
				curr_point.x = 2.9 * i;
				curr_point.y = 2.9 * j;
				curr_point.z = 0;
				temp_3d.push_back(curr_point);
			}
		}
		points_3d.push_back(temp_3d);
		frame_corners.clear();
		i--;
	}
	calibrateCamera(points_3d, corners_2d, frame.size(), intrinsic, distortionCoeffs, rvecs, tvecs);
	vector<Vec3d> rvecs2, tvecs2;
    while (1) {
		
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        //cv::Mat image = ardrone.getImage();
		cap >> image;
		//cvtColor(image, image, CV_RGB2GRAY);
		Mat out;
		initUndistortRectifyMap(intrinsic, distortionCoeffs, Mat(), intrinsic, image.size(), CV_32FC1, outputMapX, outputMapY);
		remap(image, image, outputMapX, outputMapY, INTER_LINEAR);

		cout << "fucking" << endl;

		cv::aruco::detectMarkers(image, dictionary, corners, ids);
		cv::aruco::estimatePoseSingleMarkers(corners, markerLength, intrinsic, distortionCoeffs, rvecs2, tvecs2);
		cout << "test1" << endl;
		aruco::drawDetectedMarkers(image, corners, ids);
		cout << "test2" << endl;
		//cout << rvecs2[0] << endl;
		//aruco::drawAxis(out, intrinsic, distortionCoeffs, rvecs2, tvecs2, 0.1);
		for (int index_marker = 0; index_marker < ids.size(); index_marker++)
		{
			cout << "rvecs2[0]" << rvecs2[index_marker] << endl;
			aruco::drawAxis(image, intrinsic, distortionCoeffs, rvecs2[index_marker], tvecs2[index_marker], 10);
			cout << "tvecs2[0]" << tvecs2[index_marker] << endl;
			cout << "tvecs2[1]" << tvecs2[index_marker + 1] << endl;
			cout << "tvecs2[2]" << tvecs2[index_marker + 2] << endl;
			
			cout << index_marker << endl;
		}

		cout << "fuckyou" << endl;
		//cout << tvecs2[0]<<endl;
		//cout << "fuckyou2" << endl;
        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'j') vy =  1.0;
        if (key == 'l') vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;
        ardrone.move3D(vx, vy, vz, vr);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    //ardrone.close();

    return 0;
}

