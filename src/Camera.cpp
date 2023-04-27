#include "Camera.h"

template <typename T>
void writeVectorOfVector(cv::FileStorage& fs, const std::string& name, std::vector<std::vector<T>>& data) {
	fs << name;
	fs << "{";
	for (int i = 0; i < data.size(); i++) {
		fs << name + "_" + std::to_string(i);
		std::vector<T> temp = data[i];
		fs << temp;
	}
	fs << "}";
}

template <typename T>
void readVectorOfVector(cv::FileStorage& fns, const std::string& name, std::vector<std::vector<T>>& data) {
	data.clear();
	cv::FileNode fn = fns[name];
	if (fn.empty()) {
		return;
	}

	cv::FileNodeIterator current_iter = fn.begin();
	cv::FileNodeIterator iter_end = fn.end();
	for (; current_iter != iter_end; ++current_iter) {
		std::vector<T> temp;
		cv::FileNode item = *current_iter;
		item >> temp;
		data.push_back(temp);
	}
}


Camera::Camera(const cv::String& filename) : cam_intrinsics_(cv::Matx33f::eye()), dist_coeffs_(0, 0, 0, 0, 0) {
	cv::FileStorage file(filename, cv::FileStorage::READ);
	file["Intrinsics"] >> cam_intrinsics_;
	file["Distortion"] >> dist_coeffs_;
	std::cout << cam_intrinsics_ << std::endl;
	std::cout << dist_coeffs_ << std::endl;
}

void Camera::Calibrate(const cv::String& path, const cv::String& filename, const cv::String& img_points_file, const cv::String& obj_points_file) {
	std::vector<cv::String> file_names;
	cv::glob(path, file_names, false);
	cv::Size pattern_size(8, 5);
	image_points_.resize(file_names.size());
	std::vector<cv::Point3f> obj_points;

	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	cv::FileStorage points(img_points_file, cv::FileStorage::WRITE);
	cv::FileStorage points_3d(obj_points_file, cv::FileStorage::WRITE);

	const int chess_board[2] = { 9, 6 };
	const int field_size = 30;
	for (int i = 1; i < chess_board[1]; i++) {
		for (int j = 1; j < chess_board[0]; j++) {
			obj_points.push_back(cv::Point3f(j * field_size, i * field_size, 0));
		}
	}

	std::size_t i = 0;
	for (auto const& f : file_names) {
		std::cout << std::string(f) << std::endl;

		image_ = cv::imread(file_names[i]);

		cv::cvtColor(image_, grayscale_, cv::COLOR_RGB2GRAY);

		bool pattern_found = cv::findChessboardCorners(grayscale_, pattern_size, image_points_[i], cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

		if (pattern_found) {
			cv::cornerSubPix(grayscale_, image_points_[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.001));
			object_points_.push_back(obj_points);
		}
		cv::drawChessboardCorners(image_, pattern_size, image_points_[i], pattern_found);
		cv::namedWindow("Chessboard detection", cv::WINDOW_FREERATIO);
		cv::imshow("Chessboard detection", image_);
		cv::waitKey(0);
		i++;
	}
	cv::destroyWindow("Chessboard detection");
	writeVectorOfVector(points, "values", image_points_);
	writeVectorOfVector(points_3d, "values", object_points_);
	points.release();
	points_3d.release();
	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<double> std_intrinsics, std_extrinsics;
	cv::Size frame_size(1920, 1080);
	std::vector<cv::Point3f> new_obj_points;
	std::cout << "Calibrating..." << std::endl;
	double errorRO = cv::calibrateCameraRO(object_points_, image_points_, frame_size, 1, cam_intrinsics_, dist_coeffs_, rvecs, tvecs, new_obj_points);
	//float error = cv::calibrateCamera(object_points_, image_points_, frame_size, cam_intrinsics_, dist_coeffs_, rvecs, tvecs);
	std::cout << "Reprojection error: " << errorRO << "\nK:\n" << cam_intrinsics_ << "\nk:\n" << dist_coeffs_ << std::endl;
	std::vector<cv::Point2f> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	std::vector<float> perViewErrors;
	perViewErrors.resize(object_points_.size());

	/*for (size_t i = 0; i < object_points_.size(); ++i) {
		projectPoints(object_points_[i], rvecs[i], tvecs[i], cam_intrinsics_, dist_coeffs_, imagePoints2);
		err = norm(image_points_[i], imagePoints2, cv::NORM_L2);

		size_t n = object_points_[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		std::cout << perViewErrors[i] << "\tError:\t" << file_names[i] << std::endl;
	}*/
	fs << "Intrinsics" << cam_intrinsics_;
	fs << "Distortion" << dist_coeffs_;
	//cv::Mat map_x, map_y;
	//cv::initUndistortRectifyMap(cam_intrinsics_, dist_coeffs_, cv::Mat::eye(cv::Size(3, 3), CV_32F), cam_intrinsics_, frame_size, CV_32FC1, map_x, map_y);

	/*for (auto const& f : file_names) {
		std::cout << std::string(f) << std::endl;
		cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
		cv::Mat img_undistorted, new_camera_matrix;
		cv::Size img_size = cv::Size(1920, 1080);
		//cv::remap(img, img_undistorted, map_x, map_y, cv::INTER_LINEAR);
		new_camera_matrix = cv::getOptimalNewCameraMatrix(cam_intrinsics_, dist_coeffs_, img_size, 1, img_size, 0);

		cv::undistort(img, img_undistorted, new_camera_matrix, dist_coeffs_, new_camera_matrix);

		cv::namedWindow("Undistorted image", cv::WINDOW_FREERATIO);
		cv::namedWindow("Chessboard detection", cv::WINDOW_FREERATIO);
		cv::imshow("Chessboard detection", img);
		cv::imshow("Undistorted image", img_undistorted);
		cv::waitKey(0);
	}*/
}

void Camera::CalibrateCharuco(const cv::String& path) {
	std::vector<cv::String> file_names;
	cv::glob(path, file_names, false);
	image_points_.resize(file_names.size());
	const int squares_x = 11;
	const int squares_y = 8;
	const float square_length = 0.021;
	const float marker_length = 0.017;
	std::vector<cv::Point3f> obj_points;
	std::vector<cv::Mat> allImgs;
	cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(11, 8, 0.021, 0.017, dict);
	cv::Size image_size(1920, 1080);
	std::size_t i = 0;
	obj_points = board->chessboardCorners;
	
	cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
	params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
	for (auto const& f : file_names) {
		std::cout << std::string(f) << std::endl;
		image_ = cv::imread(file_names[i]);
		std::vector<int> marker_ids;
		std::vector<std::vector<cv::Point2f>> marker_corners;
		cv::aruco::detectMarkers(image_, board->dictionary, marker_corners, marker_ids, params);

		if (marker_ids.size() > 0) {
			cv::aruco::drawDetectedMarkers(image_, marker_corners, marker_ids);
			std::vector<cv::Point2f> charuco_corners;
			std::vector<int> charuco_ids;
			cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image_, board, charuco_corners, charuco_ids);
			// if at least one charuco corner detected
			if (charuco_ids.size() > 0) {
				cv::aruco::drawDetectedCornersCharuco(image_, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));
				
				object_points_.push_back(obj_points);
			}
			charuco_corners_.push_back(charuco_corners);
			charuco_ids_.push_back(charuco_ids);
		}
		cv::namedWindow("Charuco detection", cv::WINDOW_FREERATIO);
		cv::imshow("Charuco detection", image_);
		cv::waitKey(0);
		i++;
	}
	std::vector<cv::Mat> rvecs, tvecs;
	double repError = cv::aruco::calibrateCameraCharuco(charuco_corners_, charuco_ids_, board, image_size, cam_intrinsics_, dist_coeffs_, rvecs, tvecs);
	std::cout << "Reprojection error: " << repError << "\nK:\n" << cam_intrinsics_ << "\nk:\n" << dist_coeffs_ << std::endl;
}

void CalibrateStereo(Camera& camera1, Camera& camera2, const cv::String& filename, bool is_first) {
	cv::Mat essent_mat, fund_mat;
	cv::Matx33f rot;
	cv::Matx31f trns;
	int flags = cv::CALIB_FIX_INTRINSIC;
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!is_first) {
		cv::FileStorage first_img_points("imagePointsFirst.yaml", cv::FileStorage::READ);
		cv::FileStorage first_obj_points("objectPointsFirst.yaml", cv::FileStorage::READ);
		cv::FileStorage second_img_points("imagePointsSecond.yaml", cv::FileStorage::READ);
		cv::FileStorage second_obj_points("objectPointsSecond.yaml", cv::FileStorage::READ);
		readVectorOfVector(first_img_points, "values", camera1.image_points_);
		first_img_points.release();
		readVectorOfVector(first_obj_points, "values", camera1.object_points_);
		first_obj_points.release();
		readVectorOfVector(second_img_points, "values", camera2.image_points_);
		second_img_points.release();
		readVectorOfVector(second_obj_points, "values", camera2.object_points_);
		second_obj_points.release();
	}

	double stereo_error = cv::stereoCalibrate(camera1.object_points_, camera1.image_points_, camera2.image_points_,
		camera1.cam_intrinsics_, camera1.dist_coeffs_,
		camera2.cam_intrinsics_, camera2.dist_coeffs_, camera1.grayscale_.size(),
		rot, trns, essent_mat, fund_mat, flags, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.001));
	std::cout << rot << std::endl;
	std::cout << trns << std::endl;
	std::cout << "Stereo error: " << stereo_error << std::endl;
	cv::Matx33f cam1_rot(1, 0, 0,
						 0, 1, 0,
						 0, 0, 1);
	cv::Matx31f cam1_trns(0,
						  0,
						  0);

	cv::Matx31f cam2_trns(-201.62062,
		0.6191784,
		-0.1714314);

	cv::hconcat(cam1_rot, cam1_trns, camera1.cam_extrinsics_);
	cv::hconcat(rot, trns, camera2.cam_extrinsics_);
	camera1.projection_mat_ = camera1.cam_intrinsics_ * camera1.cam_extrinsics_;
	camera2.projection_mat_ = camera2.cam_intrinsics_ * camera2.cam_extrinsics_;

	fs << "R " << rot;
	fs << "T " << trns;

}

bool EquateVectors(std::vector<std::vector<cv::Point2f>>& corners, std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners2, std::vector<int>& ids2) {
	std::vector<std::vector<cv::Point2f>> smaller_temp;
	std::vector<std::vector<cv::Point2f>> bigger_temp;
	std::vector<std::vector<cv::Point2f>> bigger_corners;
	std::vector<std::vector<cv::Point2f>> smaller_corners;
	std::vector<int> smaller_temp_ids;
	std::vector<int> bigger_temp_ids;
	std::vector<int>::const_iterator bigger_ids_begin;
	std::vector<int>::const_iterator smaller_ids_begin;
	std::vector<int>::const_iterator bigger_ids_end;	
	std::vector<int>::const_iterator smaller_ids_end;
	std::cout << ids.size() << "Size: " << ids2.size() << std::endl;
	if (ids.empty() || ids2.empty()) {
		return false;
	}
	std::cout << "Size is bigger than zero" << std::endl;
	if (ids.size() <= ids2.size()) {
		smaller_ids_begin = ids.begin();
		bigger_ids_begin = ids2.begin();
		smaller_ids_end = ids.end();
		bigger_ids_end = ids2.end();
		smaller_corners = corners;
		bigger_corners = corners2;
	} else {
		smaller_ids_begin = ids2.begin();
		bigger_ids_begin = ids.begin();
		smaller_ids_end = ids2.end();
		bigger_ids_end = ids.end();
		smaller_corners = corners2;
		bigger_corners = corners;
	}
	std::vector<int>::const_iterator bigger_ids_iter = bigger_ids_begin;
	for (auto smaller_ids_iter = smaller_ids_begin; smaller_ids_iter != smaller_ids_end; ++smaller_ids_iter) {
		if (*smaller_ids_iter != *bigger_ids_iter) {
			std::vector<int>::const_iterator temp_iter;
			temp_iter = std::find(bigger_ids_begin, bigger_ids_end, *smaller_ids_iter);
			if (temp_iter != bigger_ids_end) {
				smaller_temp.push_back(smaller_corners[smaller_ids_iter - smaller_ids_begin]);
				bigger_temp.push_back(bigger_corners[temp_iter - bigger_ids_begin]);
				smaller_temp_ids.push_back(*smaller_ids_iter);
				bigger_temp_ids.push_back(*smaller_ids_iter);
			}
		} else {
			smaller_temp_ids.push_back(*smaller_ids_iter);
			bigger_temp_ids.push_back(*smaller_ids_iter);
			smaller_temp.push_back(smaller_corners[smaller_ids_iter - smaller_ids_begin]);
			bigger_temp.push_back(bigger_corners[bigger_ids_iter - bigger_ids_begin]);
			bigger_ids_iter++;
		}
	}
	/*for (auto ids_iter = ids.begin(); (ids_iter != ids.end()) && (ids2_iter != ids2.end()); ++ids_iter, ++ids2_iter) {
		if (*ids_iter != *ids2_iter) {
			temp_ids.push_back(*ids_iter);
			std::vector<int>::iterator temp_iter = ids2_iter;
			temp_iter = std::find(ids2.begin(), ids2.end(), *ids_iter);
			if (temp_iter != ids2.end()) {
				temp.push_back(corners2[temp_iter - ids2.begin()]);
			} else {
				temp_ids.pop_back();
			}
		} else {
			temp_ids.push_back(*ids_iter);
			temp.push_back(corners2[ids_iter - ids.begin()]);
			++ids2_iter;
		}
	}*/
	if (smaller_temp_ids.empty() || bigger_temp_ids.empty()) {
		return false;
	}
	if (ids.size() <= ids2.size()) {
		corners = smaller_temp;
		corners2 = bigger_temp;
		ids = smaller_temp_ids;
		ids2 = bigger_temp_ids;
	} else {
		corners = bigger_temp;
		corners2 = smaller_temp;
		ids = bigger_temp_ids;
		ids2 = smaller_temp_ids;
	}
	return true;
}

void FindMarkers(const cv::String& filename, const Camera& camera1, const Camera& camera2) {
	cv::VideoCapture input1(2, 0);
	input1.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
	input1.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
	input1.set(cv::CAP_PROP_FPS, 30);
	cv::VideoCapture input2(1, 0);
	input2.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
	input2.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
	input2.set(cv::CAP_PROP_FPS, 30);
	if ((!input1.isOpened()) || (!input2.isOpened())) {
		return;
	}
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	//std::vector<std::vector<std::vector<cv::Point2f>>> camera1_corners;
	//std::vector<std::vector<int>> camera1_ids;
	//std::vector<std::vector<std::vector<cv::Point2f>>> camera2_corners;
	//std::vector<std::vector<int>> camera2_ids;
	//double totalTime = 0;
	//int totalIterations = 0;
	cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
	//std::vector<cv::String> file_names;
	//std::vector<cv::String> file_names2;
	//cv::glob("C:/Users/Michael Sk/Pictures/SplitCam/1/marker/1/image*.jpg", file_names, false);
	//cv::glob("C:/Users/Michael Sk/Pictures/SplitCam/1/marker/2/image*.jpg", file_names2, false);

	while (input1.grab() && input2.grab()) {
	//for (auto const& f : file_names) {
		//std::cout << std::string(f) << std::endl;
		//std::cout << std::string(file_names2[i]) << std::endl;
		cv::Mat image1, image_copy1;
		cv::Mat image2, image_copy2;
		//image1 = cv::imread(file_names[i]);
		//image2 = cv::imread(file_names2[i]);

		input1.retrieve(image1);
		input2.retrieve(image2);
		std::vector<int> ids;
		std::vector<int> ids2;
		std::vector<std::vector<cv::Point2f>> corners, rejected;
		std::vector<std::vector<cv::Point2f>> corners2, rejected2;
		//std::vector<cv::Vec3d> camera1_rvecs, camera1_tvecs;
		//std::vector<cv::Vec3d> camera2_rvecs, camera2_tvecs;
		//std::vector<cv::Vec3d> marker_centers, marker_centers2;
		//cv::undistort(image1, image_copy1, camera1.cam_intrinsics_, camera1.dist_coeffs_);
		//cv::undistort(image2, image_copy2, camera2.cam_intrinsics_, camera2.dist_coeffs_);
		cv::aruco::detectMarkers(image1, dictionary, corners, ids, detector_params, rejected);
		cv::aruco::detectMarkers(image2, dictionary, corners2, ids2, detector_params, rejected2);
		std::map<int, std::vector<cv::Point2f>> camera1_map;
		std::transform(ids.begin(), ids.end(), corners.begin(), std::inserter(camera1_map, camera1_map.end()), [](int a, std::vector<cv::Point2f>& b)
			{
				return std::make_pair(a, b);
			});
		// if at least one marker detected
		if ((ids.size() > 0) && (ids2.size() > 0)) {
			bool is_equal = false;
			cv::aruco::drawDetectedMarkers(image1, corners, ids);
			cv::aruco::drawDetectedMarkers(image2, corners2, ids2);
			//cv::imshow("Chess", image_copy1);
			//cv::imshow("Chess2", image_copy2);
			for (size_t i = 0; i < ids.size(); i++) {
				std::cout << ids[i] << "\n";
				std::cout << corners[i] << "\n";
			}
			std::cout << "\n";
			std::cout << "------------------------------------" << std::endl;
			for (auto it = camera1_map.cbegin(); it != camera1_map.cend(); ++it) {
				std::cout << it->first << " " << it->second[0] << "\n";
			}
			std::cout << "\n";
			std::cout << "Start" << std::endl;
			//is_equal = EquateVectors(corners, ids, corners2, ids2);
			std::cout << "Finish" << std::endl;
			for (size_t i = 0; i < ids.size(); i++) {
				std::cout << ids[i] << "\t";
			}
			std::cout << "\n";
			for (size_t i = 0; i < ids2.size(); i++) {
				std::cout << ids2[i] << "\t";
			}
			std::cout << "\n";
			//std::cout << corners[0][0] << std::endl;
			is_equal = EquateVectors(corners, ids, corners2, ids2);
			if (is_equal) {
				//cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera1.cam_intrinsics_, camera1.dist_coeffs_, camera1_rvecs, camera1_tvecs);
				//cv::aruco::estimatePoseSingleMarkers(corners2, marker_size, camera2.cam_intrinsics_, camera2.dist_coeffs_, camera2_rvecs, camera2_tvecs);
				//marker_centers[0] = { camera1_tvecs[0][0], camera1_tvecs[0][1], camera1_tvecs[0][2] };
				//marker_centers2[0] = { camera2_tvecs[0][0], camera2_tvecs[0][1], camera2_tvecs[0][2] };
				std::cout << ids[0] << "\t" << ids2[0] << std::endl;
				//std::cout << ids[1] << "\t" << ids2[1] << std::endl;
				std::vector<cv::Mat> pnts3D(3);
				if (ids.size() > 2 && ids2.size() > 2) {
					cv::triangulatePoints(camera1.projection_mat_, camera2.projection_mat_, corners[0], corners2[0], pnts3D[0]);
					cv::triangulatePoints(camera1.projection_mat_, camera2.projection_mat_, corners[1], corners2[1], pnts3D[1]);
					cv::triangulatePoints(camera1.projection_mat_, camera2.projection_mat_, corners[2], corners2[2], pnts3D[2]);
				} else if (ids.size() > 1 && ids2.size() > 1) {
					cv::triangulatePoints(camera1.projection_mat_, camera2.projection_mat_, corners[0], corners2[0], pnts3D[0]);
					cv::triangulatePoints(camera1.projection_mat_, camera2.projection_mat_, corners[1], corners2[1], pnts3D[1]);
				} else {
					cv::triangulatePoints(camera1.projection_mat_, camera2.projection_mat_, corners[0], corners2[0], pnts3D[0]);
				}
				char key = (char)cv::waitKey(10);
				if (key == 's') {
					std::cout << pnts3D[0] << std::endl;
					cv::Point3f point = { pnts3D[0].at<float>(0, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(1, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(2, 0) / pnts3D[0].at<float>(3, 0) };
					fs << "Coordinates " << point;
					std::cout << "saved" << std::endl;
				}
				//std::cout << pnts3D[1] << std::endl;
				std::stringstream wrist;
				std::stringstream elbow;
				std::stringstream shoulder;
				std::vector<cv::Point3f> points;
				points.resize(3);
				if (ids.size() > 2 && ids2.size() > 2) {
					points[0] = { pnts3D[0].at<float>(0, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(1, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(2, 0) / pnts3D[0].at<float>(3, 0) };
					points[1] = { pnts3D[1].at<float>(0, 0) / pnts3D[1].at<float>(3, 0), pnts3D[1].at<float>(1, 0) / pnts3D[1].at<float>(3, 0), pnts3D[1].at<float>(2, 0) / pnts3D[1].at<float>(3, 0) };
					points[2] = { pnts3D[2].at<float>(0, 0) / pnts3D[2].at<float>(3, 0), pnts3D[2].at<float>(1, 0) / pnts3D[2].at<float>(3, 0), pnts3D[2].at<float>(2, 0) / pnts3D[2].at<float>(3, 0) };
					//putText(image1, cv::format("Right wrist(%f,%f,)", points[0].x, points[0].y), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
					if (ids[0] == 21) {
						cv::putText(image1, cv::format("Right wrist: (%f,%f, %f)", points[0].x, points[0].y, points[0].z), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						if (ids[1] == 0) {
							cv::putText(image1, cv::format("Right elbow: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
							cv::putText(image1, cv::format("Right shoulder: (%f,%f, %f)", points[2].x, points[2].y, points[2].z), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						} else {
							cv::putText(image1, cv::format("Right shoulder: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
							cv::putText(image1, cv::format("Right elbow: (%f,%f, %f)", points[2].x, points[2].y, points[2].z), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						}
					} else if (ids[0] == 14) {
						cv::putText(image1, cv::format("Right shoulder: (%f,%f, %f)", points[0].x, points[0].y, points[0].z), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						if (ids[1] == 0) {
							cv::putText(image1, cv::format("Right elbow: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
							cv::putText(image1, cv::format("Right wrist: (%f,%f, %f)", points[2].x, points[2].y, points[2].z), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						} else {
							cv::putText(image1, cv::format("Right wrist: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
							cv::putText(image1, cv::format("Right elbow: (%f,%f, %f)", points[2].x, points[2].y, points[2].z), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						}
					} else {
						cv::putText(image1, cv::format("Right elbow: (%f,%f, %f)", points[0].x, points[0].y, points[0].z), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						if (ids[1] == 14) {
							cv::putText(image1, cv::format("Right wrist: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
							cv::putText(image1, cv::format("Right shoulder: (%f,%f, %f)", points[2].x, points[2].y, points[2].z), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						} else {
							cv::putText(image1, cv::format("Right shoulder: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
							cv::putText(image1, cv::format("Right wrist: (%f,%f, %f)", points[2].x, points[2].y, points[2].z), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						}
					}
				} else if (ids.size() > 1 && ids2.size() > 1) {
					points[0] = { pnts3D[0].at<float>(0, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(1, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(2, 0) / pnts3D[0].at<float>(3, 0) };
					points[1] = { pnts3D[1].at<float>(0, 0) / pnts3D[1].at<float>(3, 0), pnts3D[1].at<float>(1, 0) / pnts3D[1].at<float>(3, 0), pnts3D[1].at<float>(2, 0) / pnts3D[1].at<float>(3, 0) };
					if (ids[0] == 14) {
						cv::putText(image1, cv::format("Right wrist: (%f,%f, %f)", points[0].x, points[0].y, points[0].z), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						if (ids[1] == 0) {
							cv::putText(image1, cv::format("Right elbow: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						} else {
							cv::putText(image1, cv::format("Right shoulder: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						}
					} else if (ids[0] == 21) {
						cv::putText(image1, cv::format("Right shoulder: (%f,%f, %f)", points[0].x, points[0].y, points[0].z), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						if (ids[1] == 0) {
							cv::putText(image1, cv::format("Right elbow: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						} else {
							cv::putText(image1, cv::format("Right wrist: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(50, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						}
					} else {
						cv::putText(image1, cv::format("Right elbow: (%f,%f, %f)", points[0].x, points[0].y, points[0].z), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						if (ids[1] == 14) {
							cv::putText(image1, cv::format("Right wrist: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(70, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						} else {
							cv::putText(image1, cv::format("Right shoulder: (%f,%f, %f)", points[1].x, points[1].y, points[1].z), cv::Point(70, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
						}
					}
				} else {
					points[0] = { pnts3D[0].at<float>(0, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(1, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(2, 0) / pnts3D[0].at<float>(3, 0) };
					if (ids[0] == 14) {
						cv::putText(image1, cv::format("Right wrist: (%f,%f, %f)", points[0].x, points[0].y, points[0].z), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
					} else if (ids[0] == 21) {
						cv::putText(image1, cv::format("Right shoulder: (%f,%f, %f)", points[0].x, points[0].y, points[0].z), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
					} else {
						cv::putText(image1, cv::format("Right elbow: (%f,%f, %f)", points[0].x, points[0].y, points[0].z), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 255, 204), 1, false);
					}
				}
				//points[0] = {pnts3D[0].at<float>(0, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(1, 0) / pnts3D[0].at<float>(3, 0), pnts3D[0].at<float>(2, 0) / pnts3D[0].at<float>(3, 0)};
				//points[1] = {pnts3D[1].at<float>(0, 0) / pnts3D[1].at<float>(3, 0), pnts3D[1].at<float>(1, 0) / pnts3D[1].at<float>(3, 0), pnts3D[1].at<float>(2, 0) / pnts3D[1].at<float>(3, 0)};
				//points[2] = {pnts3D[2].at<float>(0, 0) / pnts3D[2].at<float>(3, 0), pnts3D[2].at<float>(1, 0) / pnts3D[2].at<float>(3, 0), pnts3D[2].at<float>(2, 0) / pnts3D[2].at<float>(3, 0)};
				//points.push_back
				//cv::Point3f point2 = { pnts3D[1].at<float>(0, 1) / pnts3D[1].at<float>(3, 1), pnts3D[1].at<float>(1, 1) / pnts3D[1].at<float>(3, 1), pnts3D[1].at<float>(2, 1) / pnts3D[1].at<float>(3, 1) };
				//std::cout << points[0] << std::endl;
				//std::cout << points[1] << std::endl;
				//std::cout << points[2] << std::endl;
				//std::cout << point2 << std::endl;
			}
			//i++;
		}
		cv::imshow("out", image1);
		cv::imshow("out2", image2);
		char key = (char)cv::waitKey(10);
		if (key == 27) {
			break;
		}
	}
	fs.release();
}
