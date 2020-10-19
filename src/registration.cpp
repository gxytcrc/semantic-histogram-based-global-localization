#include "registration.hpp"

registration::registration(vector<vector<float> > centerpoint1, vector<vector<float> > centerpoint2, MatrixXi matcherID){
    int size = matcherID.rows();
    sourcePoints.resize(size, 3);
    targetPoints.resize(size, 3);
    labelVector.resize(size, 1);
    totalID.resize(size, 2);
    totalID = matcherID;
    //generate the source cloud and target cloud
    for(int i=0; i<size; i++){
        int row1 = matcherID(i, 0);
        int row2 = matcherID(i, 1);

        sourcePoints(i, 0) = centerpoint1[row1][0];
        sourcePoints(i, 1) = centerpoint1[row1][1];
        sourcePoints(i, 2) = centerpoint1[row1][2];

        targetPoints(i, 0) = centerpoint2[row2][0];
        targetPoints(i, 1) = centerpoint2[row2][1];
        targetPoints(i, 2) = centerpoint2[row2][2];

        labelVector(i) = centerpoint1[row1][3];
    }
}

registration::~registration(){
    cout<<"object registration destroyed"<<endl;
}

void getTransform(MatrixXf source, MatrixXf target, MatrixXf& R, MatrixXf& T, int iteNum){

    int size = source.rows();
    MatrixXf avergae_source(1, 3);
    MatrixXf avergae_target(1, 3);
    MatrixXf converage_source(size, 3);
    MatrixXf converage_target(size, 3);
    MatrixXf A(3, 3);

    MatrixXf R0(3, 3);
    MatrixXf T0(3, 1);

    R.resize(3, 3);
    T.resize(3, 1);

    R.setIdentity();
    T.setZero();

    for(int ite=0; ite<iteNum; ite++){

        //initiallize some values
        avergae_source.setZero();
        avergae_target.setZero();
        converage_source.setZero();
        converage_target.setZero();
        
        //calculate the average value
        for(int i=0; i<size; i++){

            avergae_source(0, 0) = avergae_source(0, 0) + (source(i, 0)/size);
            avergae_source(0, 1) = avergae_source(0, 1) + (source(i, 1)/size);
            avergae_source(0, 2) = avergae_source(0, 2) + (source(i, 2)/size);

            avergae_target(0, 0) = avergae_target(0, 0) + (target(i, 0)/size);
            avergae_target(0, 1) = avergae_target(0, 1) + (target(i, 1)/size);
            avergae_target(0, 2) = avergae_target(0, 2) + (target(i, 2)/size);

        }

        //cout<<"source Average: "<<avergae_source<<endl;
        //cout<<"target Average: "<<avergae_target<<endl;

        //obtain the convaraience value of the points
        for(int i=0; i<size; i++){

            converage_source(i, 0) = source(i, 0) - avergae_source(0, 0);
            converage_source(i, 1) = source(i, 1) - avergae_source(0, 1);
            converage_source(i, 2) = source(i, 2) - avergae_source(0, 2);

            converage_target(i, 0) = target(i, 0) - avergae_target(0, 0);
            converage_target(i, 1) = target(i, 1) - avergae_target(0, 1);
            converage_target(i, 2) = target(i, 2) - avergae_target(0, 2);
        }

        //cout<<"source Converage: "<<converage_target<<endl;
        //cout<<"target Converage: "<<converage_target<<endl;

        A = converage_source.transpose() * converage_target;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
        R0 = V * U.transpose();
        T0 = avergae_target.transpose() - R0 * (avergae_source.transpose());

        for (int i = 0; i < source.rows(); i++) {
            source.row(i) = (R0*source.row(i).transpose() + T0).transpose();
        }
        R = R0 * R;
        T = T0 + T;

    }

}

void getWeightedTransform(MatrixXf source, MatrixXf target, MatrixXf& R, MatrixXf& T, VectorXf labelVector, int iteNum){

    int size = source.rows();
    MatrixXf avergae_source(1, 3);
    MatrixXf avergae_target(1, 3);
    MatrixXf A(3, 3);

    MatrixXf R0(3, 3);
    MatrixXf T0(3, 1);

    R.resize(3, 3);
    T.resize(3, 1);

    R.setIdentity();
    T.setZero();

    int count = 0;
    for(int i =0; i<size; i++){
        if(labelVector(i) == 5){
            count++;
        }
    }
    cout<<"count: "<<count<<endl;

    int new_size = size + count*10;
    MatrixXf new_source(new_size, 3);
    MatrixXf new_target(new_size, 3);
    MatrixXf converage_source(new_size, 3);
    MatrixXf converage_target(new_size, 3);

    count = 0;
    for(int i =0; i<size; i++){
        if(labelVector(i) == 5 ){
            for(int num = 0; num<10; num++){
                int currentpose = size + count*10 + num;
                new_source(currentpose, 0) = source(i, 0);
                new_source(currentpose, 1) = source(i, 1);
                new_source(currentpose, 2) = source(i, 2);

                new_target(currentpose, 0) = target(i, 0);
                new_target(currentpose, 1) = target(i, 1);
                new_target(currentpose, 2) = target(i, 2);
            }
            count++;
        }     
    }
    for(int i =0; i<size; i++){
        new_source(i, 0) = source(i, 0);
        new_source(i, 1) = source(i, 1);
        new_source(i, 2) = source(i, 2);

        new_target(i, 0) = target(i, 0);
        new_target(i, 1) = target(i, 1);
        new_target(i, 2) = target(i, 2);

    }

    for(int i =0; i<new_size; i++){
        if(i < size){
            cout<<"source: "<<source(i, 0)<<" "<<source(i, 1)<<" "<<source(i, 2)<<endl;
            cout<<"target: "<<target(i, 0)<<" "<<target(i, 1)<<" "<<target(i, 2)<<endl;
        }
        cout<<"New_source: "<<new_source(i, 0)<<" "<<new_source(i, 1)<<" "<<new_source(i, 2)<<endl;
        cout<<"New_target: "<<new_target(i, 0)<<" "<<new_target(i, 1)<<" "<<new_target(i, 2)<<endl;
    }
    for(int ite=0; ite<iteNum; ite++){

        //initiallize some values
        avergae_source.setZero();
        avergae_target.setZero();
        converage_source.setZero();
        converage_target.setZero();
        
        //calculate the average value

        for(int i=0; i<new_size; i++){

            avergae_source(0, 0) = avergae_source(0, 0) + (new_source(i, 0)/new_size);
            avergae_source(0, 1) = avergae_source(0, 1) + (new_source(i, 1)/new_size);
            avergae_source(0, 2) = avergae_source(0, 2) + (new_source(i, 2)/new_size);

            avergae_target(0, 0) = avergae_target(0, 0) + (new_target(i, 0)/new_size);
            avergae_target(0, 1) = avergae_target(0, 1) + (new_target(i, 1)/new_size);
            avergae_target(0, 2) = avergae_target(0, 2) + (new_target(i, 2)/new_size);

        }

        //cout<<"source Average: "<<avergae_source<<endl;
        //cout<<"target Average: "<<avergae_target<<endl;

        //obtain the convaraience value of the points
        for(int i=0; i<new_size; i++){

            converage_source(i, 0) = new_source(i, 0) - avergae_source(0, 0);
            converage_source(i, 1) = new_source(i, 1) - avergae_source(0, 1);
            converage_source(i, 2) = new_source(i, 2) - avergae_source(0, 2);

            converage_target(i, 0) = new_target(i, 0) - avergae_target(0, 0);
            converage_target(i, 1) = new_target(i, 1) - avergae_target(0, 1);
            converage_target(i, 2) = new_target(i, 2) - avergae_target(0, 2);
        }

        //cout<<"source Converage: "<<converage_target<<endl;
        //cout<<"target Converage: "<<converage_target<<endl;

        A = converage_source.transpose() * converage_target;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
        R0 = V * U.transpose();
        T0 = avergae_target.transpose() - R0 * (avergae_source.transpose());

        for (int i = 0; i < new_source.rows(); i++) {
            new_source.row(i) = (R0*new_source.row(i).transpose() + T0).transpose();
        }
        R = R0 * R;
        T = T0 + T;

    }

}

void registration::matcherRANSAC(float Td){
    //generate the random points for Ransac processing
    MatrixXf sourceRandom(4, 3);
    MatrixXf targetRandom(4, 3);
    //get the time value for obtaining the random points
    clock_t startTime,endTime;
    startTime = clock();

    int totalNum = sourcePoints.rows();
    int maxCount = 0;

    //initialize the inlier ID
    inlierID.resize(1,2);
    inlierID.setZero();
    for(int iter=0; iter<1000; iter++){
        endTime = clock();
        int timedifference = (int)((endTime - startTime)*10000);
        srand((int)time(0)+timedifference);

        //obtain the random points
        int Count = sourcePoints.rows();
        for(int i=0; i<4; i++){
            int randomNum = rand()%Count;
            sourceRandom.row(i) = sourcePoints.row(randomNum);
            targetRandom.row(i) = targetPoints.row(randomNum);
        }

        //calculate the R and T
        MatrixXf R, T;
        getTransform(sourceRandom, targetRandom, R, T, 1);
        // cout<<"R:"<<endl;
        // cout<<R<<endl;
        // cout<<"T: "<<endl;
        // cout<<T<<endl;

        //find the inliers
        int inliersCount = 0;
        vector<int> inliers;
        for(int i=0; i<sourcePoints.rows(); i++){
            MatrixXf newSource(1, 3);
            MatrixXf newDifference(1, 3);
            newSource = (R*sourcePoints.row(i).transpose() + T).transpose();
            //calculate the distance
            newDifference = targetPoints.row(i) - newSource;
            float distance = sqrt(pow(newDifference(0, 0), 2) + pow(newDifference(0, 1), 2) + pow(newDifference(0, 2), 2));
            if(distance<Td){
                inliersCount = inliersCount + 1;
                inliers.push_back(i);
            }
        }

        if(inliersCount<=maxCount){
            continue;   
        }
        //get the new best solusion
        maxCount = inliersCount;
    
        //if the inliers are not enough, continue search
        if(inliersCount<0.01*totalNum || inliersCount<5){
            continue;
        }

        inlierID.setZero();
        inlierID.resize(inliersCount,2);
        inlierTotalID.setZero();
        inlierTotalID.resize(inliersCount);
        for(int i=0; i<inliersCount; i++){
            int oldID = inliers[i];
            inlierTotalID(i) = oldID;
            inlierID(i, 0) = totalID(oldID, 0);
            inlierID(i, 1) = totalID(oldID, 1);
        }
    }
    double AllCount = (double)totalNum;
    double inliCount = (double)(inlierID.rows());
    double performance = inliCount/AllCount;
    cout<<"inlier num: "<<inliCount<<endl;
    cout<<"matching performance: "<<performance<<endl;

}

void registration::Alignment(){

    int size = inlierID.rows();
    sourceInliers.resize(size, 3);
    targetInliers.resize(size, 3);
    for(int i=0; i<size; i++){

        int oldID = inlierTotalID(i);

        sourceInliers.row(i) = sourcePoints.row(oldID);

        targetInliers.row(i) = targetPoints.row(oldID);
    }

    getTransform(sourceInliers, targetInliers, Rotation, Translation, 1000);
    //getWeightedTransform(sourceInliers, targetInliers, Rotation, Translation, labelVector, 1000)
    cout<<"R: "<<Rotation<<endl;
    cout<<"T: "<<Translation<<endl;

    Matrix3d Rotation0;
    Rotation0.setIdentity();
    for(int row=0; row<3; row++){
        for(int col=0; col<3; col++){
            Rotation0(row,col) = (double)Rotation(row, col);
        }
    }
    Eigen::Vector3d eulerAngle = Rotation0.eulerAngles(2, 1, 0);
    eulerAngle(0) = (eulerAngle(0)/3.1415926)*180;
    eulerAngle(1) = (eulerAngle(1)/3.1415926)*180;
    eulerAngle(2) = (eulerAngle(2)/3.1415926)*180;
    //cout<<"Eular angle: "<<eulerAngle<<endl;

}