#ifndef pointCloudMapping_hpp
#define pointCloudMapping_hpp

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/flann.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>  
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <iostream>  
#include <stdio.h>
#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>
#include<vector>
#include "toolfile.hpp"



using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;

struct pointCloudMapping
{
public:
    
    void insertValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud, Mat&depthmap, Mat&image1, Matrix4f&TransformsValue, vector<float>&camera, float scale, vector<vector<float> > Centerpoint, Mat Labels);
    void initialize3Dmap(int type);
    void pointCloudFilter();
    void pointExtraction(vector<uchar> label);
    void pointExtraction(vector<uchar> label, Mat labelimag);
    void pointVisuallize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud2, MatrixXi matcherID, MatrixXf R, MatrixXf T);
    vector<vector<float> > Cpoint;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPointCloud();
    
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalCloud;
    Matrix4f T = Matrix4f::Identity();
    Mat depth;
    Mat image;
    Mat LabelValue;
    float fx; 
    float fy; 
    float cx; 
    float cy;
    float baseline;
    float s;         
};

#endif /* pointCloudMapping */
