#ifndef  vo_hpp
#define vo_hpp

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>



#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>

#include "pointCloudMapping.hpp"


using namespace cv;
using namespace std;
using namespace Eigen;

void insertPose(string&dir, Mat &pose, int number);
void insertSYNTIAPose(string&dir, Mat&pose, int number);

cv::Mat Quaternion2Matrix (cv::Mat q);

void gatherPointCloudData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector<vector<float> >& centerpoint, Mat pose, Mat Label, vector<uchar> label_gray, vector<float> camera, float scale, string dir, int fileNumber, int startpoint);
void gatherDenseMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector<vector<float> >& centerpoint, Mat pose, Mat Label, vector<float> camera, float scale, string dir, int fileNumber, int type);
void calculateGroundTruth(Mat pose1, Mat pose2, MatrixXf Rotation, MatrixXf translation, Matrix4f& T);
void matchingEvaluation(vector<vector<float> > Cpoint1, vector<vector<float> > Cpoint2, MatrixXi matcherID, MatrixXi InlierID, float& Pvalue, float& Rvalue, float& Pcount1);
void outputCenterpoint(vector<vector<float> >& centerpoint, string txtname);
void inputCenterpoint(vector<vector<float> >& centerpoint, int number, string txtname);


#endif /* pointCloudMapping */
