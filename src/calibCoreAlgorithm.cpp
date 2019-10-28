
#include "SharedHead.h"
#include "calibCoreAlgorithm.h"

using namespace cv;
using namespace std;

bool calibCoreAlgorithm::doSingleCalibrateFishEye() {
    //step-1:read config files
    std::cout << "get in read config files" << endl;
    cv::FileStorage fs;
    fs.open(_configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open" << _configPath << std::endl;
        return false;
    }

    std::cout << "read imageType " << std::endl;

    //0:yuv /1:png/jpg
    int imageType = (int) fs["imageType"];

    double square_size=(double)fs["square_size"];


    cv::FileNode image_size = fs["image_size"];
    cv::FileNodeIterator it_initImg_size = image_size.begin();
    int image_size_width = (int) (*it_initImg_size)["width"];
    int image_size_height = (int) (*it_initImg_size)["height"];


    cv::FileNode board_size = fs["board_size"];
    cv::FileNodeIterator it_board_size = board_size.begin();
    int board_width = (int) (*it_board_size)["width"];
    int board_height = (int) (*it_board_size)["height"];
    _calibrationData._boardSize = cv::Size(board_width, board_height);

    std::cout << "read resultStorage" << std::endl;
    int resultStorage = (int) fs["resultStorage"];

    if(_camera._cameraL&&_camera._cameraR==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathL"];
        std::cout<<" _local_image_path"<< _local_image_path<<endl;

        //read serial num
        _serial_num=(string)fs["serial_numL"];
        std::cout<<"_serial_num"<<_serial_num<<endl;
    }
    if(_camera._cameraR&&_camera._cameraL==false)
    {
        //read local image_path
        _local_image_path=(string)fs["local_imagePathR"];
        std::cout<<" _local_image_path"<< _local_image_path<<endl;

        //read serial num
        _serial_num=(string)fs["serial_numR"];
        std::cout<<"_serial_num"<<_serial_num<<endl;
    }
    _resultPath=(string)fs["resultPath"];
    //cameraMatrix
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));

    //cameraDistortion
    cv::Mat distCoeffs = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));

    std::cout << "read config success.." << std::endl;
    fs.release();


    //step-2:read image
    ifstream f_img;
    f_img.open(_local_image_path.c_str());
    std::cout << "_local_image_path== " << _local_image_path.c_str() << endl;
    if (!f_img) {
        std::cerr << "Failed to open" << _local_image_path << endl;
        return false;
    }

    //read image to vector<Mat>
    std::vector<cv::Mat> imageVec;
    cv::Mat src_image;
    while (!f_img.eof()) {
        string s;
        getline(f_img, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            string img_path, img_prefix, img_name;
            ss >> img_prefix;
            ss >> img_name;
            img_path = img_prefix + "/" + img_name;

            FILE *f;
            unsigned char *pBuf = NULL;
            if (imageType == 0) {
                if (!(f = fopen(img_path.c_str(), "rb"))) {
                    std::cerr << "Failed to open " << img_path << std::endl;

                    return false;
                }
                pBuf = new unsigned char[image_size_width * image_size_height * 2];

                fseek(f, 0, SEEK_SET);
                int readfile = fread(pBuf, 1, image_size_width * image_size_height * 2, f);
                src_image = cv::Mat(image_size_height + image_size_height / 2, image_size_width, CV_8UC1, pBuf);
                cv::cvtColor(src_image, src_image, CV_YUV420sp2RGB);
            } else {
                src_image = cv::imread(img_path);
                if (src_image.empty()) {
                    std::cerr << "Failed to open" << img_path << std::endl;
                    return false;
                }

            }
            imageVec.push_back(src_image);
        }//if empty
    }//while
    f_img.close();

    std::cout << "imageVec.size()== " << imageVec.size() << std::endl;

    if (imageVec.size() == 0) {
        std::cerr << "no image to detect corners " << std::endl;
        return false;
    }

    pair<int, std::vector<cv::Point2d>> pair_corner_pnt2ds_image;
    std::vector<std::vector<cv::Point2d>> image_corner_pnt2ds_vec;
    image_corner_pnt2ds_vec.clear();

    //vector<cv::Point2f> image_corner_pnts;
    vector<cv::Point2d> image_corner_pnt2ds;
    vector<cv::Point2f> image_corner_pnt2fs;
    //step-3: detect corners
    for (int image_count = 0; image_count < imageVec.size(); ++image_count) {
        _image_count = (image_count + 1); //save detected image needed
        image_corner_pnt2ds.clear();
        image_corner_pnt2fs.clear();
        //for fish eye
        if (!calibCoreAlgorithm::detectCornersFishEye(imageVec[image_count], _calibrationData._boardSize,
                                                      image_corner_pnt2ds)) {
            continue;
        }
        image_corner_pnt2ds_vec.push_back(image_corner_pnt2ds);
        pair_corner_pnt2ds_image.first = image_count;
        pair_corner_pnt2ds_image.second = image_corner_pnt2ds;
        _pair_corner_pnt2ds_image_vec.push_back(pair_corner_pnt2ds_image);
    }

    std::cout << "image detected success num= " << _pair_corner_pnt2ds_image_vec.size() << std::endl;


    _calibrationData._image_corner_pnt2ds_vec = image_corner_pnt2ds_vec;

    //step-4:create corner in caliboard
    std::vector<std::vector<cv::Point3d>> corner_in_caliboard;
    if (!calibCoreAlgorithm::generateObjPntsInCalibBoardFishEye(_pair_corner_pnt2ds_image_vec.size(), square_size,
                                                                _calibrationData._boardSize, corner_in_caliboard,
                                                                _calibrationData._plane2dPntsVec)) {
        std::cerr << "create corner in caliboard failed" << std::endl;
        return false;
    }
    std::cout << "corner_in_caliboard== " << corner_in_caliboard.size() << std::endl;
    _calibrationData._plane3dPntsVec = corner_in_caliboard;
    //step-4: single cam calib
    std::cout << "get to calibCamera() success" << endl;
    _calibrationData._imageSize = cv::Size(imageVec[0].cols, imageVec[0].rows);
    std::cout << "image_size== " << _calibrationData._imageSize << std::endl;

    if (!calibCoreAlgorithm::ZhangCalibrationKannalaBrandt(_calibrationData, _fishEyeCamPara)) {
        return false;
    }

    //yong.qi addded for test
    std::vector<cv::Mat> imgUndistortImgVec;
    verifyCalibrationFishEyeOK(imageVec, _fishEyeCamPara, imgUndistortImgVec);

    return true;

}


bool calibCoreAlgorithm::ZhangCalibrationKannalaBrandt(CalibrationData &cData, FishEyeCamPara &camPara) {

    std::cout<<"get in ZhangCalibrationKannalaBrandt()"<<endl;

    double fishEyeCalib_rms = fisheye::calibrate(cData._plane3dPntsVec, cData._image_corner_pnt2ds_vec,
                                                 cData._imageSize, camPara._cameraMatrix, camPara._DistortCoeff,
                                                 cData._rvecsMat, cData._tvecsMat,
                                                 cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC + cv::fisheye::CALIB_FIX_SKEW,
                                                 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));

    std::cout << "ZhangCalibrationKannalaBrandt for cameraMatrix == " << camPara._cameraMatrix << std::endl;
    std::cout << "ZhangCalibrationKannalaBrandt for distortCoeff== " << camPara._DistortCoeff << std::endl;


    for (size_t h = 0; h < _calibrationData._rvecsMat.size(); h++) {
        RT rt;
        rt._R = _calibrationData._rvecsMat[h];
        rt._T = _calibrationData._tvecsMat[h];
        camPara._imgRTVec.push_back(rt);
    }

    std::cout << "cameraMatrix== " << camPara._cameraMatrix << endl;
    std::cout << "distCoeffs== " << camPara._DistortCoeff << endl;

    std::cout << "fishEyeCalib_rms== " << fishEyeCalib_rms << std::endl;

    //step-5: computeReprojectError
    if (!calibCoreAlgorithm::computeReprojectionErrorFishEye(camPara._cameraMatrix, camPara._DistortCoeff,
                                                             cData._rvecsMat, cData._tvecsMat,
                                                             cData._image_corner_pnt2ds_vec, cData._plane3dPntsVec)) {
        std::cerr << "failed to computeReprojectionError.. " << std::endl;
        return false;
    }
    std::cout << "reprojectionError[0]== " << camPara._ReprojectionError[0];
    std::cout << "reprojectionError[1]== " << camPara._ReprojectionError[1];

    double totalNum = sqrt(pow(camPara._ReprojectionError[0], 2) + pow(camPara._ReprojectionError[1], 2));

    std::cout << "totalNum== " << totalNum << std::endl;

    std::cout << "totalReproNormErr== " << camPara._totalReproNormErr << endl;

    return true;
}


cv::Mat calibCoreAlgorithm::localUndistortImgFishEye(cv::Mat img, cv::Mat mK, cv::Mat mDistCoef) {


    vector<cv::Point2f> image_points(4);
    std::vector<cv::Point2f> undistort_image_points(4);

    Point2f Point_top_left(0, 0), Point_top_right(img.cols, 0), Point_bottom_left(0, img.rows), Point_bottom_right(
            img.cols, img.rows);
    image_points[0] = Point_top_left;
    image_points[1] = Point_top_right;
    image_points[2] = Point_bottom_left;
    image_points[3] = Point_bottom_right;

    fisheye::undistortPoints(image_points, undistort_image_points, mK, mDistCoef, noArray(), mK);

    int minX = (int) min(undistort_image_points[0].x, undistort_image_points[2].x);
    int maxX = (int) max(undistort_image_points[1].x, undistort_image_points[3].x);

    int minY = (int) min(undistort_image_points[0].y, undistort_image_points[1].y);
    int maxY = (int) max(undistort_image_points[2].y, undistort_image_points[3].y);

    Size original_undistortedImage_size((int) (maxX - minX), (int) (maxY - minY));

    Size new_undistortedImage_size(img.cols, img.rows);

    Size image_size(int(img.cols), int(img.rows));

    Mat newCameraMatrix0, newCameraMatrix1, newCameraMatrix2;
    Rect validPixROI(0, 0, 0, 0);
    double balance_0 = 0, balance_1 = 1;

    vector<Rect> vValidPixROI;

    fisheye::estimateNewCameraMatrixForUndistortRectify(mK, mDistCoef, image_size, noArray(), newCameraMatrix0,
                                                        balance_0, new_undistortedImage_size, 1.0);


    fisheye::estimateNewCameraMatrixForUndistortRectify(mK, mDistCoef, image_size, noArray(), newCameraMatrix1,
                                                        balance_1, new_undistortedImage_size, 1.0);


    cv::namedWindow("srcImg_fisheye", 0);
    cv::namedWindow("undistortImg_cameraMatrix_fisheye", 0);
    //cv::namedWindow("undistortImg_newcameraMatrix0_fisheye", 0);
    cv::namedWindow("undistortImg_newcameraMatrix1_fisheye", 0);
    {
        cv::imshow("srcImg_fisheye", img);
        //std::cout<<"srcImg_fisheye src_mDistCoef== "<<mDistCoef<<endl;
        //std::cout<<"srcImg_fisheye src_mk== "<<mK<<endl<<endl<<endl;

        cv::waitKey(100);
    }

    Mat srcundistort;
    {
        cv::Mat mapx, mapy;

        cv::fisheye::initUndistortRectifyMap(mK, mDistCoef, Matx33d::eye(), mK, new_undistortedImage_size, CV_32FC1,
                                             mapx, mapy);

        //std::cout<<"undistortImg_cameraMatrix_fisheye mDistCoef== "<<mDistCoef<<endl;

        //std::cout<<"undistortImg_cameraMatrix_fisheye mK== "<<mK<<endl;

        //std::cout<<"undistortImg_cameraMatrix_fisheye new_undistortedImage_size== "<<new_undistortedImage_size<<endl<<endl<<endl;

        remap(img, srcundistort, mapx, mapy, INTER_AREA);

        cv::imshow("undistortImg_cameraMatrix_fisheye", srcundistort);

        cv::waitKey(100);
    }

//    {
//        cv::Mat mapx, mapy;
//
//        cv::fisheye::initUndistortRectifyMap(mK, mDistCoef, Matx33d::eye(), newCameraMatrix0, new_undistortedImage_size, CV_32FC1,
//                                             mapx, mapy);
//
//        remap(img, srcundistort, mapx, mapy, INTER_AREA);
//
//        cv::imshow("undistortImg_newcameraMatrix0_fisheye", srcundistort);
//
//        cv::waitKey(600);
//    }

//    {
//        cv::Mat mapx, mapy;
//
//        cv::fisheye::initUndistortRectifyMap(mK, mDistCoef, Matx33d::eye(), newCameraMatrix1, new_undistortedImage_size,
//                                             CV_32FC1,
//                                             mapx, mapy);
//       // std::cout<<"undistortImg_newcameraMatrix1_fisheye mDistCoef== "<<mDistCoef<<endl;
//       // std::cout<<"undistortImg_newcameraMatrix1_fisheye newCameraMatrix1== "<<newCameraMatrix1<<endl;
//       // std::cout<<"undistortImg_newcameraMatrix1_fisheye new_undistortedImage_size== "<<new_undistortedImage_size<<endl<<endl<<endl;
//
//        remap(img, srcundistort, mapx, mapy, INTER_AREA);
//
//        cv::imshow("undistortImg_newcameraMatrix1_fisheye", srcundistort);
//
//        cv::waitKey(100);
//    }

    return srcundistort;
}



void calibCoreAlgorithm::verifyCalibrationFishEyeOK(std::vector<cv::Mat> imgSrcVec, FishEyeCamPara &camPara,
                                                    std::vector<cv::Mat> &imgUndistortVec) {

    //请大家自行设计实验，验证我们标定的正确性（从是否已经正确矫正其畸变角度来分析）

}


bool calibCoreAlgorithm::detectCornersFishEye(const cv::Mat imageInput, cv::Size board_size,
                                              std::vector<cv::Point2d> &image_points) {
    std::cout << "board_size== " << board_size << endl;

    cv::Mat imageGray;
    if (imageInput.channels() == 1) {
        imageInput.copyTo(imageGray);
    } else {
        cvtColor(imageInput, imageGray, CV_BGR2GRAY);
    }

    //	//add blur
    cv::GaussianBlur(imageGray, imageGray, cv::Size(3, 3), 0);

    if (!findCirclesGrid(imageGray, board_size, image_points, CALIB_CB_ASYMMETRIC_GRID)) {
        std::cerr << "can not find Asymmetric CirclesGrid corners!" << std::endl;
        return false;
    }

    //yong.qi added for test
    cv::Mat colorImg;
    cv::cvtColor(imageGray, colorImg, CV_GRAY2BGR);
    for (int k = 0; k < image_points.size(); ++k) {
        std::string s = std::to_string(k + 1);
        cv::putText(colorImg, s, image_points[k], cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
    }
    //added fort test
    char imageName[50];
    if (_camera._cameraL) {
        string nameTemL = "../data/detectedL/imageDectedL";
        sprintf(imageName, "%s%d%s", nameTemL.data(), _image_count, ".bmp");
        cv::imwrite(imageName, colorImg);
    }

    return true;
}






bool calibCoreAlgorithm::generateObjPntsInCalibBoardFishEye(const int num_img_corner_detected, const double square_size,
                                                            const cv::Size board_size,
                                                            std::vector<std::vector<cv::Point3d>> &corner_in_caliboard3d,
                                                            std::vector<std::vector<cv::Point2d>> &corner_in_caliboard2d) {
    std::cout << "square_size== " << square_size << endl;

    std::cout << "board_size.height== " << board_size.height;

    std::cout << "board_size.width== " << board_size.width;

    corner_in_caliboard3d.clear();

    corner_in_caliboard2d.clear();

    vector<Point3d> point3dsTem;

    vector<Point2d> point2dsTem;

    for (size_t image_cout = 0; image_cout < num_img_corner_detected; image_cout++) {
        point3dsTem.clear();
        point2dsTem.clear();
        for (int row = 0; row < board_size.height; ++row) {
            for (int col = 0; col < board_size.width; ++col) {
                Point3d pntTem3d;
                Point2d pntTem2d;

                pntTem3d.x = (2 * col + row % 2) * square_size;
                pntTem3d.y = (row) * square_size;
                pntTem3d.z = 0;

                pntTem2d = Point2d(pntTem3d.x, pntTem3d.y);

                point3dsTem.push_back(pntTem3d);
                point2dsTem.push_back(pntTem2d);
            }
        }
        corner_in_caliboard3d.push_back(point3dsTem);
        corner_in_caliboard2d.push_back(point2dsTem);
    }
    return true;


}



bool calibCoreAlgorithm::computeReprojectionErrorFishEye(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                                         const vector<cv::Mat> rvecsMat, const vector<cv::Mat> tvecsMat,
                                                         const std::vector<std::vector<cv::Point2d>> &image_corner_pnts,
                                                         const std::vector<std::vector<cv::Point3d>> &object_points) {
    std::cout << "get in computeReprojectionError()" << endl;
    vector<vector<Point2d> > error_image_points;
    std::vector<cv::Point2d> perframe_imagepnt;
    for (int k = 0; k < object_points.size(); ++k) {
        perframe_imagepnt.clear();

        fisheye::projectPoints(object_points[k], perframe_imagepnt, rvecsMat[k], tvecsMat[k], cameraMatrix, distCoeffs);
        error_image_points.push_back(perframe_imagepnt);

        double err = cv::norm(image_corner_pnts[k], perframe_imagepnt, cv::NORM_L2);
        //std::cout << "err== " << err << endl;

    }

    vector<double> absErr;
    vector<Point2d> subDst;
    int totalPoints = 0;
    double totalError = 0;

    for (int m = 0; m < object_points.size(); ++m) {

        vector<Point2d> temp;
        cv::subtract(error_image_points[m], image_corner_pnts[m], temp);
        copy(temp.begin(), temp.end(), back_inserter(subDst));
        double err;
        int n = image_corner_pnts[m].size();
        err = norm(error_image_points[m], image_corner_pnts[m], CV_L2);
        absErr.push_back((float) sqrt(err * err / n));
        totalError += err * err;
        totalPoints += n;
    }
    totalError = std::sqrt(totalError / totalPoints);

    Scalar xmean, ymean;
    Scalar xstd_dev, ystd_dev;
    Scalar errormean, errorstd_dev;
    meanStdDev(subDst, errormean, errorstd_dev);

    _fishEyeCamPara._ReprojectionError[0] = errorstd_dev[0];
    _fishEyeCamPara._ReprojectionError[1] = errorstd_dev[1];
    _fishEyeCamPara._totalReproNormErr = totalError;

    return true;

}


bool calibCoreAlgorithm::WriteStereoCalibFilesFishEye(const std::string &calib_file_name, const FishEyeCamPara &camL,const string serial_num_L,
                                                      const FishEyeCamPara &camR,const string serial_num_R,
                                                      const RT &stereo_RT) {
    cv::FileStorage fs(calib_file_name, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }
    //Left Cam calibfiles

    fs<<"serial_num_L"<<serial_num_L;
    fs << "cam_matrix_L" << camL._cameraMatrix;
    fs << "cam_distortion_L" << camL._DistortCoeff;
    fs << "totalReproError_L" << camL._totalReproNormErr;
    fs << "ReproError_X_L" << camL._ReprojectionError[0];
    fs << "ReproError_Y_L" << camL._ReprojectionError[1];


    //Right Cam calib
    fs<<"serial_num_R"<<serial_num_R;
    fs << "cam_matrix_R" << camR._cameraMatrix;
    fs << "cam_distortion_R" << camR._DistortCoeff;
    fs << "totalReproError_R" << camR._totalReproNormErr;
    fs << "ReproError_X_R" << camR._ReprojectionError[0];
    fs << "ReproError_Y_R" << camR._ReprojectionError[1];

    //stereo cam Files
    fs<<"right_to_left"<<" ";
    fs << "system_rotation" << stereo_RT._R;
    fs << "system_translation" << stereo_RT._T;
    fs << "stereo_calib_rms" << stereo_RT._stero_rms;
    fs << "stereo_length_base" << stereo_RT._length_T;
    fs.release();
    return true;
}


