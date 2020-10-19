#ifndef registration_hpp
#define registration_hpp


#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include <iostream>  
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <Eigen/SVD> 
#include <vector>
#include <ctime>

using namespace cv;
using namespace std;
using namespace Eigen;

class registration
{
    public:
        registration(vector<vector<float> > centerpoint1, vector<vector<float> > centerpoint2, MatrixXi matcherID);
        ~registration();
        void matcherRANSAC(float Td);
        void Alignment();
        MatrixXi inlierID;
        MatrixXf Rotation, Translation;

    private:
        MatrixXf sourcePoints;
        MatrixXf targetPoints;
        MatrixXf sourceInliers;
        MatrixXf targetInliers;
        MatrixXi totalID;
        VectorXi inlierTotalID;
        VectorXi labelVector;

};

void getTransform(MatrixXf source, MatrixXf target);
#endif /* pointCloudMapping */
