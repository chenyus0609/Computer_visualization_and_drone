#include "ardrone/ardrone.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include "pid.h"
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
	Mat intrinsic;
	Mat distortionCoeffs;
	Mat outputMapX;
	Mat outputMapY;

	//detect marker
	vector<Vec3d> rvecs;
	vector<Vec3d> tvecs;
	const float markerLength = 12.7;
	vector<int> ids;
	vector<vector<Point2f>> corners;

	//dictionary
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	FileStorage fs("calibration.xml", FileStorage::READ);
	fs["intrinsic"] >> intrinsic;
	fs["distortion"] >> distortionCoeffs;

	// Initialize
	if (!ardrone.open()) {
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
	std::cout << "***************************************" << std::endl;


	//PID controls
	PIDManager my_pids("pid.yaml");
	cv::Mat error(4, 1, CV_64F);
	cv::Mat output_v(4, 1, CV_64F);
	/*output_v.at<double>(0, 0) = 0; //x
	output_v.at<double>(1, 0) = 0; //y
	output_v.at<double>(2, 0) = 0; //z
	output_v.at<double>(3, 0) = 0;    //rotation*/


	//VideoCapture cap(1);
	
	//Mat Mout;
	//initUndistortRectifyMap(intrinsic, distortionCoeffs, Mat(), intrinsic, frame.size(), CV_32FC1, outputMapX, outputMapY);
	//remap(frame, out, outputMapX, outputMapY, INTER_LINEAR);
	
	while (1) {

		// Key input
		int key = cv::waitKey(15);
		if (key == 0x1b) break;

		output_v.at<double>(0, 0) = 0; //x
		output_v.at<double>(1, 0) = 0; //y
		output_v.at<double>(2, 0) = 0; //z
		output_v.at<double>(3, 0) = 0; //rotation
		
		// Get an image
		cv::Mat image = ardrone.getImage();
		//Mat Mout;
		initUndistortRectifyMap(intrinsic, distortionCoeffs, Mat(), intrinsic, image.size(), CV_32FC1, outputMapX, outputMapY);
		remap(image, image, outputMapX, outputMapY, INTER_LINEAR);

		cv::aruco::detectMarkers(image, dictionary, corners, ids);
		cv::aruco::estimatePoseSingleMarkers(corners, markerLength, intrinsic, distortionCoeffs, rvecs, tvecs);
		aruco::drawDetectedMarkers(image, corners, ids);
		//cout << "test2" << endl;
		//cout << rvecs2[0] << endl;
		//aruco::drawAxis(out, intrinsic, distortionCoeffs, rvecs2, tvecs2, 0.1);
		for (int index_marker = 0; index_marker < ids.size(); index_marker++)
		{
			//cout << "rvecs[0]" << rvecs[index_marker] << endl;
			aruco::drawAxis(image, intrinsic, distortionCoeffs, rvecs[index_marker], tvecs[index_marker], 10);
			//cout << "tvecs[0]" << tvecs[index_marker] << endl;
			//cout << "tvecs[1]" << tvecs[index_marker + 1] << endl;
			//cout << "tvecs[2]" << tvecs[index_marker + 2] << endl;

			cout << index_marker << endl;
		}

		//cout << tvecs[0][2] << endl;

		
		//cout << tvecs2[0]<<endl;
		//cout << "fuckyou2" << endl;
		// Take off / Landing 
		if (key == ' ') {
			if (ardrone.onGround()) ardrone.takeoff();
			else                    ardrone.landing();
		}

		// Move
		double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
		if (key == 'i' || key == CV_VK_UP)    vx = 1.0;
		if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
		if (key == 'u' || key == CV_VK_LEFT)  vr = 1.0;
		if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
		if (key == 'j') vy = 1.0;
		if (key == 'l') vy = -1.0;
		if (key == 'q') vz = 1.0;
		if (key == 'a') vz = -1.0;

		//PID speed adjustment
		if (tvecs.size()) {
			error.at<double>(0, 0) = tvecs[0][2] - 100; //x
			//cout << "1\n";
			error.at<double>(1, 0) = tvecs[0][0]; //y
			error.at<double>(2, 0) = tvecs[0][1]; //z
			error.at<double>(3, 0) = rvecs[0][2];    //rotation
			//cout << "in tvecs size:" << tvecs.size() << "\n";
			my_pids.getCommand(error, output_v);
			cout << "output_v[0]: " << output_v.at<double>(0, 0) << endl;
			vx = output_v.at<double>(0, 0);
			vy = 0;// output_v.at<double>(1, 0);
			vz = 0;//output_v.at<double>(2, 0);
			vr = 0;// output_v.at<double>(3, 0);
		}
		//cout << "tvecs size:" << tvecs.size() << "\n";

		//cout << "fuckyou" << endl;
		//ardrone.move3D(output_v.at<double>(0, 0)*0.7, output_v.at<double>( 1,0)*0.7, output_v.at<double>(2,0)*0.7, output_v.at<double>(3,0));
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