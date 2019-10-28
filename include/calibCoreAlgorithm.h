#ifndef INTRINSIC_H_INCLUDED
#define INTRINSIC_H_INCLUDED


#include <fstream>
#include <sstream>
#include <iterator>

#include "include/SharedHead.h"

using namespace cv;
using namespace std;


class calibCoreAlgorithm {

public:
    calibCoreAlgorithm() : _image_count(0)
                           {

		_pair_corner_pnt2ds_image_vec.clear();
    }

    ~calibCoreAlgorithm() {

    }

public:

    bool doSingleCalibrateFishEye();

    bool ZhangCalibrationKannalaBrandt(CalibrationData &cData,FishEyeCamPara &camPara);

	bool detectCornersFishEye(const cv::Mat imageInput, cv::Size board_size, std::vector<cv::Point2d> &image_points);


	bool generateObjPntsInCalibBoardFishEye(const int num_img_corner_detected, const double square_size, const cv::Size board_size,
			std::vector<std::vector<cv::Point3d>> &corner_in_caliboard3d,
			std::vector<std::vector<cv::Point2d>> &corner_in_caliboard2d);

    bool computeReprojectionErrorFishEye(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                                             const vector<cv::Mat> rvecsMat, const vector<cv::Mat> tvecsMat,
                                                             const std::vector<std::vector<cv::Point2d>> &image_corner_pnts,
                                                             const std::vector<std::vector<cv::Point3d>> &object_points);

    static bool WriteStereoCalibFilesFishEye(const std::string &calib_file_name, const FishEyeCamPara &camL,const string serial_num_L,
                                             const FishEyeCamPara &camR,const string serial_num_R,
                                             const RT &stereo_RT);


    cv::Mat localUndistortImgFishEye(cv::Mat img, cv::Mat mK, cv::Mat mDistCoef);

    void
    verifyCalibrationFishEyeOK(std::vector<cv::Mat> imgSrcVec, FishEyeCamPara &camPara, std::vector<cv::Mat> &imgUndistortVec);


    void undistortImg(const cv::Mat &src, const cv::Mat &dst, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                      const cv::Mat &R);

public:

    calibCameraLR _camera; //distinguish L or R

    string _local_image_path; //image read path

    string _configPath; //ini

    string _resultPath; //calib result path to save

    string _serial_num; //serial num

    int _image_count; //calib image num


	std::vector<pair<int, vector<Point2d>>> _pair_corner_pnt2ds_image_vec; //for fisheye


    FishEyeCamPara _fishEyeCamPara; //for fish eye calib

    CalibrationData _calibrationData;


    cv::Mat _cameraMatrix_L,_DistortCoeff_L;
    cv::Mat _cameraMatrix_R,_DistortCoeff_R;

};


#endif // INTRINSIC_H_INCLUDED
