#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect.hpp>

class Camera {
private:
	cv::Matx33f cam_intrinsics_;
	cv::Matx34f cam_extrinsics_;
	cv::Matx34f projection_mat_;
	cv::Mat dist_coeffs_;
	std::vector<std::vector<cv::Point2f>> image_points_;
	std::vector<std::vector<cv::Point3f>> object_points_;
	cv::Mat image_;
	cv::Mat grayscale_;
	std::vector<std::vector<int>> charuco_ids_;
	std::vector<std::vector<cv::Point2f>> charuco_corners_;
public:
	Camera() : cam_intrinsics_(cv::Matx33f::eye()), dist_coeffs_(0, 0, 0, 0, 0) {};
	Camera(const cv::String& filename);
	void Calibrate(const cv::String& path, const cv::String& filename, const cv::String& img_points_file, const cv::String& obj_points_file);
	void CalibrateCharuco(const cv::String& path);
	friend void FindMarkers(const cv::String& filename, const Camera& camera1, const Camera& camera2);
	friend void CalibrateStereo(Camera& camera1, Camera& camera2, const cv::String& filename, bool is_first);
};

bool EquateVectors(std::vector<std::vector<cv::Point2f>>& corners, std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners2, std::vector<int>& ids2);

template <typename T>
void writeVectorOfVector(cv::FileStorage& fs, const std::string& name, std::vector<std::vector<T>>& data);
std::map<int, std::vector<cv::Point2f>>& appendElements(std::map<int, std::vector<cv::Point2f>>& current, const std::map<int, std::vector<cv::Point2f>>& previous);
template <typename T>
void readVectorOfVector(cv::FileStorage& fns, const std::string& name, std::vector<std::vector<T>>& data);
//cv::Vec3d GetCenterInCameraWorld(double marker_size, cv::Vec3d rvec, cv::Vec3d tvec);