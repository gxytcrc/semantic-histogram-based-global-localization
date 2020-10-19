#include "matcher.hpp"
#include "neighborhood.hpp"

//Random walk decriptor matching
matcher::matcher(Descriptor Ds1, Descriptor Ds2){
    cout<<"matcher constructed"<<endl;
    clock_t startTime,endTime;
    noNei = Ds1.noNeighbor;
    //the sum number of the descriptors
    size1 = Ds1.size();
    size2 = Ds2.size();
    //obtain the decriptors' matrix size
    int matrixRow = Ds1.SN;
    int matrixCol = Ds1.SL;
    int sumNumber = matrixRow;
    cout<<"Des matrixRow: "<<matrixRow<<endl;
    scoreMatrix.resize(size1, size2);
    scoreMatrix.setZero();
    for(int i = 0; i<size1; i++){
        for(int j = 0; j<size2; j++){
            MatrixXf descriptor1 = Ds1.getDescriptor(i);
            MatrixXf descriptor2 = Ds2.getDescriptor(j);
            if(descriptor1(0, 0) != descriptor2(0, 0) || descriptor1(0, 0) == 100 ){
                scoreMatrix(i, j) = 0;
                continue;
            }
            int scoreCount1 = 0;
            int scoreCount2 = 0;
            startTime = clock();
            for(int row1 = 0; row1<matrixRow; row1++){
                
                //descriptor1 matches descriptor2
                //obatin vector from the descriptor matrix1
                VectorXf vector1;
                vector1.resize(matrixCol);
                vector1 = descriptor1.row(row1);
                //cout<<"vector: "<<vector1<<endl;
                int maxCount = 0;
                for(int row2 = 0; row2<matrixRow; row2++){
                    //obatin vector from the descriptor matrix2
                    VectorXf vector2;
                    vector2.resize(matrixCol);
                    vector2 = descriptor2.row(row2);
                    //init a count number;
                    if(vector1 == vector2){
                       maxCount = 1;
                       break;
                    }
                }
                scoreCount1 = scoreCount1 + maxCount;
            }

            for(int row1 = 0; row1<matrixRow; row1++){

                //descriptor2 matches descriptor1
                //obatin vector from the descriptor matrix2
                VectorXf _vector1;
                _vector1.resize(matrixCol);
                _vector1 = descriptor2.row(row1);
                int _maxCount = 0;
                for(int row2 = 0; row2<matrixRow; row2++){
                    //obatin vector from the descriptor matrix1
                    VectorXf _vector2;
                    _vector2.resize(matrixCol);
                    _vector2 = descriptor1.row(row2);
                    //init a count number;
                    if(_vector1 == _vector2){
                        _maxCount = 1;
                        break;
                    }
                }
                scoreCount2 = scoreCount2 + _maxCount;

            }

            //use the smaller score as the final score value
            float finalScore1 = (float)scoreCount1;
            float finalScore2 = (float)scoreCount2;

            if(scoreCount1<scoreCount2){
                scoreMatrix(i, j) = finalScore1/sumNumber;
            }
            else{
                scoreMatrix(i, j) = finalScore2/sumNumber;
            }
            endTime = clock();
            double timedifference = (double)(endTime - startTime)/CLOCKS_PER_SEC;
        }
        
    }
    //cout<<scoreMatrix<<endl;
}

//FAST Descriptor matching
matcher::matcher(Descriptor Ds1, Descriptor Ds2, int type){
    if(type == 1){

        cout<<"matcher constructed"<<endl;
        noNei = Ds1.noNeighbor;
        //the sum number of the descriptors
        size1 = Ds1.size();
        size2 = Ds2.size();
        MatrixXf descriptor0 = Ds1.getDescriptor(0);
        int matrixRow = descriptor0.rows();
        int matrixCol = descriptor0.cols();
        scoreMatrix.resize(size1, size2);
        scoreMatrix.setZero();

        for(int i = 0; i<size1; i++){
            float averageDes1 = 0;
            MatrixXf descriptor1 = Ds1.getDescriptor(i);
            for(int row = 0; row<matrixRow; row++){
                for(int col = 0; col<matrixCol; col++){
                    averageDes1 = averageDes1 + descriptor1(row, col);
                }
            }
            averageDes1 = averageDes1/(matrixRow*matrixCol);
            //cout<<averageDes1<<endl;
            for(int j = 0; j<size2; j++){
                float score = 0;
                float averageDes2 = 0;
                MatrixXf descriptor2 = Ds2.getDescriptor(j);
                if(descriptor1(0,0) != descriptor2(0, 0)){
                    scoreMatrix(i, j) = 0;
                    continue;
                }
                for(int row = 0; row<matrixRow; row++){
                    for(int col = 0; col<matrixCol; col++){
                        averageDes2 = averageDes2 + descriptor2(row, col);
                    }
                }
                averageDes2 = averageDes2/(matrixRow*matrixCol);
                //caculate the corrlation
                float SumTop = 0; 
                for(int row = 0; row<matrixRow; row++){
                    for(int col = 0; col<matrixCol; col++){
                        SumTop = SumTop + ((descriptor1(row, col) - averageDes1)*(descriptor2(row, col) - averageDes2));
                    }
                }
                float SumBottomLeft = 0; 
                float SumBottomRight = 0; 
                for(int row = 0; row<matrixRow; row++){
                    for(int col = 0; col<matrixCol; col++){
                        SumBottomLeft = SumBottomLeft + (pow((descriptor1(row, col) - averageDes1),2));
                        SumBottomRight = SumBottomRight + (pow((descriptor2(row, col) - averageDes2),2));
                    }
                }
                float SumBottom = sqrt(SumBottomLeft*SumBottomRight);
                score = SumTop/SumBottom;
                scoreMatrix(i, j) = abs(score);
            }
        }

        //cout<<scoreMatrix<<endl;

    }
    else{
        cout<<"matcher constructed"<<endl;
        noNei = Ds1.noNeighbor;
        //the sum number of the descriptors
        size1 = Ds1.size();
        size2 = Ds2.size();

        scoreMatrix.resize(size1, size2);
        scoreMatrix.setZero();

        MatrixXf descriptor0 = Ds1.getDescriptor(0);
        int Num = descriptor0.rows();

        VectorXf lablevector1 = Ds1.labelVector;
        VectorXf lablevector2 = Ds2.labelVector;
        
        for(int i = 0; i<size1; i++ ){
            MatrixXf descriptor1 = Ds1.getDescriptor(i);
            int lab1 = lablevector1(i);
            if(noNei(i) == 1){
                continue;
            }
            for(int j = 0; j<size2; j++){
                float score = 0;
                MatrixXf descriptor2 = Ds2.getDescriptor(j);
                int lab2 = lablevector2(j);
                if(lab1 != lab2){
                    scoreMatrix(i, j) = 0;
                    continue;
                }
                float SumTop = 0;
                for(int row = 0; row<Num; row++){
                    SumTop = SumTop + (descriptor1(row, 0)*descriptor2(row, 0));
                }
                float SumBottomLeft = 0; 
                float SumBottomRight = 0; 
                for(int row = 0; row<Num; row++){
                    SumBottomLeft = SumBottomLeft + pow(descriptor1(row, 0), 2);
                    SumBottomRight = SumBottomRight + pow(descriptor2(row, 0), 2);
                }
                float SumBottom = sqrt(SumBottomLeft*SumBottomRight);
                score = SumTop/SumBottom;
                scoreMatrix(i, j) = abs(score);
            }
        }
        //cout<<scoreMatrix<<endl;

    }
}

matcher::~matcher(){
    cout<<"The matching object destroyed"<<endl;
}

MatrixXi matcher::getMatcherID(){
    MatrixXi matcherID(size1, 2);
    for(int i=0; i < size1; i++){
        matcherID(i, 0) = i;
        float maxScore = 0;
        int ID = 1000;
        for(int j=0; j<size2; j++){
            if(scoreMatrix(i, j)>maxScore){
                maxScore = scoreMatrix(i, j);
                ID = j;
            }
        }
        matcherID(i, 1) = ID;
    }
    return matcherID;
}

MatrixXi matcher::getGoodMatcher(){
    MatrixXi matcherID(size1, 3);
    MatrixXi goodMatcherId;
    matcherID.setZero();
    int count = 0;
    for(int i =0; i<size1; i++){
        //cout<<"current point: "<<i<<" -------------------"<<endl;
        int ID = 1000;
        float maxScore = 0;
        float secondMaxScore = 0;
        if(noNei(i) == 1){
            continue;
        }
        for(int j =0; j<size2; j++){
            if(scoreMatrix(i, j)>maxScore){
                secondMaxScore = maxScore;
                maxScore = scoreMatrix(i, j);
                ID = j;
            }

        }
        if(maxScore<0.5){
            //cout<<"no good score"<<endl;
            // cout<<"max: "<<maxScore<<endl;
            // cout<<"secondMax: "<<secondMaxScore<<endl;
            continue;
        }
        
        int InverseID = 100000;
        float InverseScore = 0;
        // cout<<"matchedID: "<<ID<<endl;
        for(int IDi =0; IDi<size1; IDi++){
            // cout<<IDi<<endl;
            // cout<<"current score: "<<scoreMatrix(IDi, ID)<<endl;
            if(scoreMatrix(IDi, ID)>InverseScore){
                InverseScore = scoreMatrix(IDi, ID);
                InverseID = IDi;
                //cout<<"maxmum score: "<<InverseScore<<endl;
            }

        }
        if(InverseID != i){
            //cout<<"no good match"<<endl;
            // cout<<"ID: "<<InverseID<<endl;
            // cout<<"max: "<<InverseScore<<endl;
            //continue;
        }
        matcherID(i, 1) = ID;
        matcherID(i, 2) = 1;
        count = count+1;
    }

    goodMatcherId.resize(count, 2);
    count = 0;
    for(int i =0; i<size1; i++){
        if(matcherID(i, 2) != 1){
            continue;
        }
        goodMatcherId(count, 0) = i;
        goodMatcherId(count, 1) = matcherID(i, 1);
        count = count+1;
    }
    return goodMatcherId;
}
