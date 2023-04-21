#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include "Camera.h"

const int SQUARE_SIZE = 2;
const int BOARD_ROWS = 6;
const int BOARD_COLS = 9;

void ZhangCalibration(const cv::String& path);


int main() {
	Camera first;
	Camera second;
	/*cv::Mat m(1920, 1080, CV_8UC1);
	cv::Ptr< cv::aruco::Dictionary > dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
	cv::Ptr< cv::aruco::CharucoBoard > charuco = cv::aruco::CharucoBoard::create(9, 7, 0.02, 0.015, dict);
	charuco->draw(cv::Size(1920, 1080), m, 0, 1);
	cv::imwrite("board.png", m);*/
	//first.CalibrateCharuco("C:/Users/Michael Sk/Pictures/SplitCam/1/marker2/15/image*.jpg");
	//second.CalibrateCharuco("C:/Users/Michael Sk/Pictures/SplitCam/1/marker2/16/image*.jpg");
	//cv::Ptr<cv::aruco::CharucoBoard> board = ... // create charuco board
	//cv::Size imgSize(1920, 1080); // camera image size
	//std::vector<std::vector<cv::Point2f>> allCharucoCorners;
	//std::vector<std::vector<int>> allCharucoIds;
	// Detect charuco board from several viewpoints and fill allCharucoCorners and allCharucoIds
		// After capturing in several viewpoints, start calibration
		//cv::Mat cameraMatrix, distCoeffs;
	//std::vector<cv::Mat> rvecs, tvecs;
	//int calibrationFlags = ... // Set calibration flags (same than in calibrateCamera() function)
	//	double repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);
	first.Calibrate("C:/Users/Michael Sk/Pictures/SplitCam/1/marker2/17/image*.jpg");
	second.Calibrate("C:/Users/Michael Sk/Pictures/SplitCam/1/marker2/18/image*.jpg");
	CalibrateStereo(first, second);
	FindMarkers("markers.yaml", first, second);
	//ZhangCalibration("C:/Users/Michael Sk/Pictures/SplitCam/1/old/image*.jpg");
	/*cv::VideoCapture webcam(1, 0);
	webcam.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	webcam.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
	webcam.set(cv::CAP_PROP_FPS, 30);
	if (!webcam.isOpened()) {
		return -1;
	}
	std::vector<std::vector<cv::Point3f>> objs_points;
	std::vector<cv::Point3f> object_points;
	std::vector<std::vector<cv::Point2f>> image_points;

	for (size_t i = 0; i < BOARD_ROWS; i++) {
		for (size_t j = 0; j < BOARD_COLS; j++) {
			object_points.push_back(cv::Point3f(i * SQUARE_SIZE, j * SQUARE_SIZE, 0));
		}
	}
	std::vector<cv::Point2f> corner_points;
	while (webcam.grab()) {
		cv::Mat image, image_copy, temp_image;
		webcam.retrieve(image);
		temp_image = image.clone();
		cv::cvtColor(image, image_copy, cv::COLOR_BGR2GRAY);
		cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
		bool pattern_found = cv::findChessboardCorners(image_copy, cv::Size(BOARD_ROWS, BOARD_COLS), corner_points, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
		if (pattern_found) {
			cv::cornerSubPix(image_copy, corner_points, cv::Size(11, 11), cv::Size(-1, -1), criteria);
			cv::drawChessboardCorners(temp_image, cv::Size(BOARD_ROWS, BOARD_COLS), corner_points, pattern_found);
			cv::imshow("Video", temp_image);
			/*if (key == 27) {
				break;
			}
			if (key == 'c') {
				image_points.push_back(corner_points);
				objs_points.push_back(object_points);
			}
		} else {
			cv::imshow("Video", image);
		}
		char key = (char)(cv::waitKey(1000 / 30));
	}*/
	/*cv::Mat K(3, 3, CV_32F);
	cv::Vec<float, 5> k(0, 0, 0, 0, 0);
	std::vector<cv::Mat> rvecs, tvecs;
	int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
	cv::calibrateCamera(objs_points, image_points, cv::Size(1280, 720), K, k, rvecs, tvecs, flags);
	std::cout << "cameraMatrix : " << K << std::endl;*/
	return 0;
}

void ZhangCalibration(const cv::String& path) {
	cv::Matx33f K;
	cv::Vec<float, 5> k;
	std::vector<cv::String> file_names;
	cv::glob(path, file_names, false);
	cv::Size pattern_size(9, 7);
	std::vector<std::vector<cv::Point2f>> q(file_names.size());
	std::vector<std::vector<cv::Point3f>> Q;

	int chess_board[2] = { 10, 8 };
	int field_size = 21;
	std::vector<cv::Point3f> object_points;
	for (int i = 1; i < chess_board[1]; i++) {
		for (int j = 1; j < chess_board[0]; j++) {
			object_points.push_back(cv::Point3f(j * field_size, i * field_size, 0));
		}
	}

	std::vector<cv::Point3f> image_points;
	std::size_t i = 0;
	for (auto const& f : file_names) {
		std::cout << std::string(f) << std::endl;

		cv::Mat image = cv::imread(file_names[i]);
		cv::Mat gray;

		cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);

		bool pattern_found = cv::findChessboardCorners(gray, pattern_size, q[i], cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

		if (pattern_found) {
			cv::cornerSubPix(gray, q[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			
			Q.push_back(object_points);
		}
		cv::drawChessboardCorners(image, pattern_size, q[i], pattern_found);
		cv::namedWindow("Chessboard detection", cv::WINDOW_FREERATIO);
		cv::imshow("Chessboard detection", image);
		cv::waitKey(0);
		i++;
	}

	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<double> std_intrinsics, std_extrinsics;
	int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_K3 + cv::CALIB_FIX_PRINCIPAL_POINT;
	cv::Size frame_size(1920, 1080);

	std::cout << "Calibrating..." << std::endl;

	float error = cv::calibrateCamera(Q, q, frame_size, K, k, rvecs, tvecs, flags);
	std::cout << "Reprojection error: " << error << "\nK:\n" << K << "\nk:\n" << k << std::endl;;

	cv::Mat map_x, map_y;
	cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frame_size, CV_32FC1, map_x, map_y);

	for (auto const& f : file_names) {
		std::cout << std::string(f) << std::endl;

		cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

		cv::Mat img_undistorted;
		cv::remap(img, img_undistorted, map_x, map_y, cv::INTER_LINEAR);
		cv::namedWindow("Undistorted image", cv::WINDOW_FREERATIO);
		cv::imshow("Undistorted image", img_undistorted);
		cv::waitKey(0);
	}
}