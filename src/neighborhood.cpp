#include "neighborhood.hpp"

Neighborhood::Neighborhood(vector<vector<float> > Cpoint){
    cout<<"Neighborhood construct"<<endl;
    int size = Cpoint.size();
    label.resize(size);
    for(int i =0; i<size; i++){
        vector<float> cp(4);
        cp[0] = Cpoint[i][0]; cp[1] = Cpoint[i][1]; cp[2] = Cpoint[i][2]; cp[3] = Cpoint[i][3]; 
        label(i) = cp[3];
        centerpoint.push_back(cp);
    }
}

Neighborhood::~Neighborhood(){
    cout<<"The Neighborhood object destroyed"<<endl;
}

MatrixXf Neighborhood::getNeighbor(){

    MatrixXf neighbor(centerpoint.size(), centerpoint.size());
    neighbor.setZero();
    for(int n = 0; n < centerpoint.size(); n++){
        float x0 = centerpoint[n][0];
        float y0 = centerpoint[n][1];
        float z0 = centerpoint[n][2];
        for(int m = n+1; m<centerpoint.size(); m++){
            float x1 = centerpoint[m][0];
            float y1 = centerpoint[m][1];
            float z1 = centerpoint[m][2];
            float distance = sqrt(pow(x0-x1,2)+ pow(y0-y1,2) + pow(z0-z1,2));
            // cout<<"label0: "<<centerpoint[n][3]<<endl;
            // cout<<"label1: "<<centerpoint[m][3]<<endl;
            // cout<<"distance: "<<distance<<endl;

            if(distance<20){
                neighbor(m, n) = 1;
                neighbor(n, m) = 1;
            }
        }
    }
    return neighbor;

}

//-----------------------------------------------------------------------------------------------
// Random walk descriptor
Descriptor::Descriptor(Neighborhood Nei, int stepNumber, int stepLengh){
    clock_t startTime,endTime;
    startTime = clock();
    cout<<"Descriptor construct"<<endl;
    //obtain the neibore relations;
    neighbor = Nei.getNeighbor();
    labelVector = Nei.label;
    cout<<"vector size: "<<labelVector.size()<<endl;
    //cout<<labelVector<<endl;
    //inite the no neighbor vector;
    noNeighbor.resize(labelVector.size());
    noNeighbor.setZero();
    SN = stepNumber;
    SL = stepLengh;
    endTime = clock();
    int timedifference = (int)((endTime - startTime)*10000);
    int size = neighbor.rows();
    cout<<"time1: "<<time(0)<<endl;
    srand((int)time(0)+timedifference);
    cout<<"test1"<<endl;
    for(int row = 0; row<size; row++){

        MatrixXf SingleDescriptor(SN, SL);
        float label;
        float PointID;
        for(int stepNum = 0; stepNum<SN; stepNum++){
            for(int stepDepth = 0; stepDepth<SL; stepDepth++){
                if(stepDepth == 0){
                    PointID = row;
                }
                label = labelVector(PointID);
                SingleDescriptor(stepNum, stepDepth) = label;
                
                //find the neighbor point
                int count = 0;
                vector<int> neighborID;
                for(int IDnum = 0; IDnum<size; IDnum++){
                    if(neighbor(PointID, IDnum) == 1){
                        neighborID.push_back(IDnum);
                        count = count+1;
                    }
                }
                if(count == 0){
                    noNeighbor(PointID) = 1;
                    continue;
                }
                int pathID = rand()%count;
                PointID = neighborID[pathID];
            }
        }

        DescriptorVector.push_back(SingleDescriptor);
    }


}
//FAST Descriptor
Descriptor::Descriptor(Neighborhood Nei){
    cout<<"Descriptor construct"<<endl;
    neighbor = Nei.getNeighbor();
    labelVector = Nei.label;
    //inite the no neighbor vector;
    noNeighbor.resize(labelVector.size());
    noNeighbor.setZero();
    int size = neighbor.rows();
    int labelSize = 11;
    for(int Num =0; Num<size; Num++){
        MatrixXf SingleDescriptor(labelSize, 1);
        SingleDescriptor.setZero();
        //colmum 2
        //find the neibor point
        int neiCount = 0;
        vector<int> neighborID;
        for(int NeiNum = 0; NeiNum<size; NeiNum++){
            if(neighbor(Num, NeiNum) == 1){
                int locallabel = labelVector(NeiNum);
                SingleDescriptor(locallabel, 0) = SingleDescriptor(locallabel, 0) + 1;
                neighborID.push_back(NeiNum);
                neiCount = neiCount+1;
            }
        }
        if(neiCount ==0){
            noNeighbor(Num) = 1;
            DescriptorVector.push_back(SingleDescriptor);
            continue;
        }
        DescriptorVector.push_back(SingleDescriptor);
    }
}

Descriptor::Descriptor(Neighborhood Nei, int Histogram){
    cout<<"Descriptor construct"<<endl;
    neighbor = Nei.getNeighbor();
    labelVector = Nei.label;
    //inite the no neighbor vector;
    noNeighbor.resize(labelVector.size());
    noNeighbor.setZero();
    int size = neighbor.rows();
    int labelSize = 11;
    for(int Num =0; Num<size; Num++){
        MatrixXf SingleDescriptor(1331, 1);
        SingleDescriptor.setZero();
        int VectorNum = 0;
        //colmum 2
        //find the neibor point
        int neiCount = 0;
        vector<int> neighborID;
        for(int NeiNum = 0; NeiNum<size; NeiNum++){
            if(neighbor(Num, NeiNum) == 1){
                neighborID.push_back(NeiNum);
                neiCount = neiCount+1;
            }
        }
        if(neiCount ==0){
            noNeighbor(Num) = 1;
            DescriptorVector.push_back(SingleDescriptor);
            continue;
        }
        //colmum 3
        int pointlable = labelVector(Num);
        for(int NeiID =0; NeiID<neiCount; NeiID++){

            int NeiNum = neighborID[NeiID];
            int previoudlable = labelVector(NeiNum);

            for(int NeiNeiNum = 0; NeiNeiNum<size; NeiNeiNum++){
                if(neighbor(NeiNum, NeiNeiNum) == 1){
                    int locallabel = labelVector(NeiNeiNum);
                    int VectorNum = pointlable * 121 + previoudlable*11 + locallabel;
                    SingleDescriptor(VectorNum, 0) = SingleDescriptor(VectorNum, 0) + 1;
                }
            }
        }
        DescriptorVector.push_back(SingleDescriptor);
    }
}

Descriptor::~Descriptor(){
    cout<<"The Descriptor object destroyed"<<endl;
}

MatrixXf Descriptor::getDescriptor(int DesID){
    MatrixXf SingleDescriptor;
    SingleDescriptor = DescriptorVector[0];
    int rows =  SingleDescriptor.rows();
    int cols = SingleDescriptor.cols();
    
    int des_size = DescriptorVector.size();
    //cout<<"des size: "<<des_size<<endl;
    for(int i = 0; i<des_size; i++){
        if(i == DesID){
            return DescriptorVector[i];
        }
    }
    cout<<"error, no this ID's Descriptor"<<endl;
    MatrixXf errorMat(rows,cols);
    errorMat.setZero();
    return errorMat;
}

//can only draw the RandomWalk descriptor
void Descriptor::draw(int DesID, Mat lab){
    int row = SN*100;
    int col = SL*100;
    Mat DescriptorPlot = cv::Mat::zeros(row, col, CV_8UC3);
    MatrixXf singleDescriptor = DescriptorVector[DesID];
    for(int i =0; i<SN; i++){
        for(int j=0; j<SL; j++){
            int startPointi = i*100;
            int endPointi = i*100+99;
            int startPointj = j*100;
            int endPointj = j*100+99;
            int label = (int)singleDescriptor(i, j);
            int R = lab.at<int>(label, 0);
            int G = lab.at<int>(label, 1);
            int B = lab.at<int>(label, 2);
            for(int pointi = startPointi; pointi<endPointi; pointi++){
                for(int pointj = startPointj; pointj<endPointj; pointj++){
                    DescriptorPlot.at<Vec3b>(pointi, pointj)[0] = B;
                    DescriptorPlot.at<Vec3b>(pointi, pointj)[1] = G;
                    DescriptorPlot.at<Vec3b>(pointi, pointj)[2] = R;
                }
            }
        }
    }
    imshow("descriptor", DescriptorPlot);
    waitKey(1000000);
}

int Descriptor::size(){
    int des_size = DescriptorVector.size();
    return des_size;
}
