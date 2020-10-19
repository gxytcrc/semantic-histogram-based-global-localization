#ifndef neighborhood_hpp
#define neighborhood_hpp

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include <iostream>  
#include <stdio.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <pangolin/pangolin.h>
#include <vector>

#include <cstdlib>
#include <ctime>


using namespace cv;
using namespace std;
using namespace Eigen;

class Neighborhood
{
    public:
        Neighborhood(vector<vector<float> > Cpoint);
        ~Neighborhood();
        MatrixXf getNeighbor();
        VectorXf label;
    private:
        vector<vector<float> > centerpoint;
        
};

class Descriptor
{
    public:
        Descriptor(Neighborhood Nei, int stepNumber, int stepLengh);
        Descriptor(Neighborhood Nei);
        Descriptor(Neighborhood Nei, int Histogram);
        ~Descriptor();
        MatrixXf getDescriptor(int DesID);
        void draw(int DesID, Mat lab);
        int size();
        int SL;
        int SN;
        VectorXf noNeighbor;
        VectorXf labelVector;


    private:
        MatrixXf neighbor;
        vector<MatrixXf> DescriptorVector;
};



#endif /* pointCloudMapping */
