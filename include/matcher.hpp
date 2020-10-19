#ifndef matcher_hpp
#define matcher_hpp

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include <iostream>  
#include <stdio.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <vector>
#include "neighborhood.hpp"


using namespace cv;
using namespace std;
using namespace Eigen;

class matcher
{
    public:
        matcher(Descriptor Ds1, Descriptor Ds2);
        matcher(Descriptor Ds1, Descriptor Ds2, int type);
        ~matcher();
        MatrixXf scoreMatrix;
        MatrixXi getMatcherID();
        MatrixXi getGoodMatcher();

    private:
        int size1;
        int size2;
        VectorXf noNei;
};

#endif /* pointCloudMapping */

