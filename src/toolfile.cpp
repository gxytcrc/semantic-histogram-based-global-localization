#include "toolfile.hpp"

void insertPose(string&dir, Mat&pose, int number){

    double a[number];
    double b[number];
    double c[number];
    double d[number];
    double e[number];
    double f[number];
    double g[number];

    string txtname = dir+"airsim.txt";
    ifstream myfile(txtname);
    int N;  
    myfile>>N;
    int count = 0;
    for(int i=0;i<number;i++) 
    {
        myfile>>a[i]>>b[i]>>c[i]>>d[i]>>e[i]>>f[i]>>g[i];
        //cout<<a[i]<<" "<<b[i]<<" "<<c[i]<<" "<<d[i]<<" "<<e[i]<<" "<<f[i]<<" "<<g[i]<<endl;
        //cout<<" "<<endl;
        pose.at<double>(i, 0) = a[i];
        pose.at<double>(i, 1) = b[i];
        pose.at<double>(i, 2) = c[i];
        pose.at<double>(i, 3) = d[i];
        pose.at<double>(i, 4) = e[i];
        pose.at<double>(i, 5) = f[i];
        pose.at<double>(i, 6) = g[i];

        
    } 
    myfile.close();    
}

void insertSYNTIAPose(string&dir, Mat&pose, int number){

    double a[number];
    double b[number];
    double c[number];
    double d[number];
    double e[number];
    double f[number];
    double g[number];
    double h[number];
    double i1[number];
    double j[number];
    double k[number];
    double l[number];
    double m[number];
    double n[number];
    double o[number];
    double p[number];

    string txtname = dir+"synthia.txt";
    ifstream myfile(txtname);
    int N;  
    myfile>>N;
    int count = 0;
    for(int i=0;i<number;i++) 
    {
        myfile>>a[i]>>b[i]>>c[i]>>d[i]>>e[i]>>f[i]>>g[i]>>h[i]>>i1[i]>>j[i]>>k[i]>>l[i]>>m[i]>>n[i]>>o[i]>>p[i];
        //cout<<a[i]<<" "<<b[i]<<" "<<c[i]<<" "<<d[i]<<" "<<e[i]<<" "<<f[i]<<" "<<g[i]<<endl;
        //cout<<" "<<endl;
        pose.at<double>(i, 0) = a[i];
        pose.at<double>(i, 1) = b[i];
        pose.at<double>(i, 2) = c[i];
        pose.at<double>(i, 3) = d[i];
        pose.at<double>(i, 4) = e[i];
        pose.at<double>(i, 5) = f[i];
        pose.at<double>(i, 6) = g[i];
        pose.at<double>(i, 7) = h[i];                
        pose.at<double>(i, 8) = i1[i];
        pose.at<double>(i, 9) = j[i];
        pose.at<double>(i, 10) = k[i];
        pose.at<double>(i, 11) = l[i];
        pose.at<double>(i, 12) = m[i];
        pose.at<double>(i, 13) = n[i];
        pose.at<double>(i, 14) = o[i];
        pose.at<double>(i, 15) = p[i];


        
    } 
    myfile.close();    
}

void outputCenterpoint(vector<vector<float> >& centerpoint, string txtname){
    ofstream outfile;
    outfile.open(txtname);
    int size = centerpoint.size();
    for(int i =0; i<size; i++){
        float x = centerpoint[i][0];
        float y = centerpoint[i][1];
        float z = centerpoint[i][2];
        float l = centerpoint[i][3];
        outfile<<x<<" "<<y<<" "<<z<<" "<<l<<endl;        
    }
    outfile.close();
}

void inputCenterpoint(vector<vector<float> >& centerpoint, int number, string txtname){
    ifstream myfile(txtname);
    int N;  
    myfile>>N;
    int count = 0;
    for(int i=0;i<number;i++) 
    {
        float x, y, z, l;
        myfile>>x>>y>>z>>l;
        vector<float> cp(4);
        cp[0] = x; cp[1] = y; cp[2] = z; cp[3] = l;
        cout<<x<<" "<<y<<" "<<z<<" "<<l<<endl;
        cout<<" "<<endl;
        centerpoint.push_back(cp);
        
    } 
    myfile.close();    
}


cv::Mat Quaternion2Matrix (cv::Mat q)
{
  double w = q.at<double>(0);
  double x = q.at<double>(1);
  double y = q.at<double>(2);
  double z = q.at<double>(3);

  double xx = x*x;
  double yy = y*y;
  double zz = z*z;
  double xy = x*y;
  double wz = w*z;
  double wy = w*y;
  double xz = x*z;
  double yz = y*z;
  double wx = w*x;

  double ret[4][4];
  ret[0][0] = 1.0f-2*(yy+zz);
  ret[0][1] = 2*(xy-wz);
  ret[0][2] = 2*(wy+xz);
  ret[0][3] = 0.0f;
 
  ret[1][0] = 2*(xy+wz);
  ret[1][1] = 1.0f-2*(xx+zz);
  ret[1][2] = 2*(yz-wx);
  ret[1][3] = 0.0f;
 
  ret[2][0] = 2*(xz-wy);
  ret[2][1] = 2*(yz+wx);
  ret[2][2] = 1.0f-2*(xx+yy);
  ret[2][3] = 0.0f;
 
  ret[3][0] = 0.0f;
  ret[3][1] = 0.0f;
  ret[3][2] = 0.0f;
  ret[3][3] = 1.0f;
 
  return cv::Mat(4,4,CV_64FC1,ret).clone();
}

Matrix4f obtainTransformMatrix(Mat pose, int i, int type){
    Matrix4f frame_pose = Matrix4f::Identity();
    if(type == 1){
        Mat q = cv::Mat::zeros(4, 1, CV_64FC1);
        q.at<double>(0,0) = pose.at<double>(i, 3);
        q.at<double>(0,1) = pose.at<double>(i, 4);
        q.at<double>(0,2) = pose.at<double>(i, 5);
        q.at<double>(0,3) = pose.at<double>(i, 6);
        Mat rotation = cv::Mat::zeros(4, 4, CV_64FC1);
        rotation = Quaternion2Matrix (q);
        for(int row = 0; row<3; row++){
            for(int col=0; col<3; col++){
                frame_pose(row, col) = rotation.at<double>(row, col);
            }
        }
        frame_pose(0,3) = pose.at<double>(i, 0);
        frame_pose(1,3) = pose.at<double>(i, 1);
        frame_pose(2,3) = pose.at<double>(i, 2);
    }
    else{

        int count = 0;
        for(int col = 0; col<4; col++){
            for(int row=0; row<4; row++){
                frame_pose(row, col) = pose.at<double>(i, count);
                count = count + 1;
            }
        }
        
    }
    return frame_pose;
}

void gatherPointCloudData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector<vector<float> >& centerpoint, Mat pose, Mat Label, vector<uchar> label_gray, vector<float> camera, float scale, string dir, int fileNumber, int startpoint){
    //init the pointcloud mapping structure 
    pointCloudMapping pointCloudMapping;
    //initiallize the frame pose
    Matrix4f frame_pose = Matrix4f::Identity();
    Matrix4f frame_pose0 = Matrix4f::Identity();
    Matrix4f camera_pose = Matrix4f::Identity();

    for(int i =startpoint; i<fileNumber; i++){
        cout<<"number: "<<i<<endl;
        if(i%3 != 0 && i !=0){
            //continue;
        }

        //load the pose of the current image
        frame_pose = obtainTransformMatrix(pose, i, 1);
        
        if(i == startpoint){
            frame_pose0 = frame_pose;
        }
        camera_pose = frame_pose0.inverse() * frame_pose;

        //load the rgb image, depth image and the segmentation image
        char base_name[256];
        sprintf(base_name,"%d.png",i);
        string depth_file_name =dir+"depth/depth_" + base_name;
        string rgb_file = dir+"segmentation/segmentation_"+base_name;
        //only SYNTHIA need
        //string label_file = dir+"label/segmentation_"+base_name;

        Mat depth = imread(depth_file_name,0);
        Mat image_rgb = imread(rgb_file);
        //only SYNTHIA need
        //Mat lab = imread(label_file,0);

        //3D recontrcute the points
        pointCloudMapping.insertValue(cloud, depth, image_rgb, frame_pose, camera, scale, centerpoint, Label);
        pointCloudMapping.pointExtraction(label_gray);
        centerpoint = pointCloudMapping.Cpoint;    
        cloud = pointCloudMapping.outputPointCloud();
        //viewer.showCloud(cloud);
        waitKey(1);    
    }

}

void gatherDenseMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector<vector<float> >& centerpoint, Mat pose, Mat Label, vector<float> camera, float scale, string dir, int fileNumber, int type){
    //init the pointcloud mapping structure 
    pointCloudMapping pointCloudMapping;
    //initiallize the frame pose
    Matrix4f frame_pose = Matrix4f::Identity();
    Matrix4f frame_pose0 = Matrix4f::Identity();
    Matrix4f camera_pose = Matrix4f::Identity();
    pcl::visualization::CloudViewer viewer("viewer");

    for(int i =0; i<fileNumber; i++){
        if(i%10 != 0 && i !=0){
            //continue;
        }

        frame_pose = obtainTransformMatrix(pose, i, 1);

        if(i == 0){
            frame_pose0 = frame_pose;
        }
        camera_pose = frame_pose0.inverse() * frame_pose;
        //load the rgb image, depth image and the segmentation image
        char base_name[256];
        sprintf(base_name,"%d.png",i);
        string depth_file_name =dir+"depth/depth_" + base_name;
        string rgb_file = dir+"segmentation/segmentation_"+base_name;
        cout<<depth_file_name<<endl;
        Mat depth = imread(depth_file_name,0);
        Mat image_rgb = imread(rgb_file);
        //3D recontrcute the points
        pointCloudMapping.insertValue(cloud, depth, image_rgb, frame_pose, camera, scale, centerpoint, Label);
        pointCloudMapping.initialize3Dmap(type);
        pointCloudMapping.pointCloudFilter();
        cloud = pointCloudMapping.outputPointCloud();
        viewer.showCloud(cloud);
    }

}

void matchingEvaluation(vector<vector<float> > Cpoint1, vector<vector<float> > Cpoint2, MatrixXi matcherID, MatrixXi InlierID, float& Pvalue, float& Rvalue, float& Pcount1){
    int size0 = matcherID.rows();
    float Pcount0 = 0; float Ncount0 = 0;
    for(int i = 0; i<size0; i++){
        VectorXf point1(3);
        VectorXf point2(3);
        int ID1 = matcherID(i, 0);
        int ID2 = matcherID(i, 1);
        point1(0) = Cpoint1[ID1][0]; point1(1) = Cpoint1[ID1][1]; point1(2) = Cpoint1[ID1][2];
        point2(0) = Cpoint2[ID2][0]; point2(1) = Cpoint2[ID2][1]; point2(2) = Cpoint2[ID2][2];
        float distance = sqrt(pow(point1(0)-point2(0),2) + pow(point1(1)-point2(1),2) + pow(point1(2)-point2(2),2));
        if(distance<10){
            Pcount0 = Pcount0 + 1;
        }
        else if(distance>=10){
            Ncount0 = Ncount0 + 1;
        }
    }
    int size1 = InlierID.rows();
    Pcount1 = 0; float Ncount1 = 0;    
    for(int i = 0; i<size1; i++){
        VectorXf point1(3);
        VectorXf point2(3);
        int ID1 = InlierID(i, 0);
        int ID2 = InlierID(i, 1);
        point1(0) = Cpoint1[ID1][0]; point1(1) = Cpoint1[ID1][1]; point1(2) = Cpoint1[ID1][2];
        point2(0) = Cpoint2[ID2][0]; point2(1) = Cpoint2[ID2][1]; point2(2) = Cpoint2[ID2][2];
        float distance = sqrt(pow(point1(0)-point2(0),2) + pow(point1(1)-point2(1),2) + pow(point1(2)-point2(2),2));
        if(distance<10){
            Pcount1 = Pcount1 + 1;
        }
        else if(distance>=10){
            Ncount1 = Ncount1 + 1;
        }
    }
    Pvalue = Pcount1/(Pcount1 + Ncount1);
    Rvalue = Pcount1/(Pcount0);
    float rate = Pcount1/(Pcount0+Ncount0);
    cout<<"Pvalue: "<<Pvalue<<endl;
    cout<<"Rvalue: "<<Rvalue<<endl;
    cout<<"PNumber: "<<Pcount1<<endl;
    cout<<"MatchRate: "<<rate<<endl;

}

void calculateGroundTruth(Mat pose1, Mat pose2, MatrixXf Rotation, MatrixXf translation, Matrix4f& T){
    Matrix4f frame_pose1 = Matrix4f::Identity();
    Matrix4f frame_pose2 = Matrix4f::Identity();
    Matrix4f GT = Matrix4f::Identity();
    Matrix4f ownPose = Matrix4f::Identity();
    
    frame_pose1(0,3) = pose1.at<double>(0, 0);
    frame_pose1(1,3) = pose1.at<double>(0, 1);
    frame_pose1(2,3) = pose1.at<double>(0, 2);
    Mat q1 = cv::Mat::zeros(4, 1, CV_64FC1);
    q1.at<double>(0,0) = pose1.at<double>(0, 3);
    q1.at<double>(0,1) = pose1.at<double>(0, 4);
    q1.at<double>(0,2) = pose1.at<double>(0, 5);
    q1.at<double>(0,3) = pose1.at<double>(0, 6);
    
    Mat rotation1 = cv::Mat::zeros(4, 4, CV_64FC1);
    rotation1 = Quaternion2Matrix (q1);
    for(int row = 0; row<3; row++){
        for(int col=0; col<3; col++){
            frame_pose1(row, col) = rotation1.at<double>(row, col);
        }
    }

    frame_pose2(0,3) = pose2.at<double>(0, 0);
    frame_pose2(1,3) = pose2.at<double>(0, 1);
    frame_pose2(2,3) = pose2.at<double>(0, 2);
    Mat q2 = cv::Mat::zeros(4, 1, CV_64FC1);
    q2.at<double>(0,0) = pose2.at<double>(0, 3);
    q2.at<double>(0,1) = pose2.at<double>(0, 4);
    q2.at<double>(0,2) = pose2.at<double>(0, 5);
    q2.at<double>(0,3) = pose2.at<double>(0, 6);
    
    Mat rotation2 = cv::Mat::zeros(4, 4, CV_64FC1);
    rotation2 = Quaternion2Matrix (q2);
    for(int row = 0; row<3; row++){
        for(int col=0; col<3; col++){
            frame_pose2(row, col) = rotation2.at<double>(row, col);
        }
    }

    GT = frame_pose1.inverse()*frame_pose2;
    
    for(int row = 0; row<3; row++){
        for(int col=0; col<3; col++){
            ownPose(row, col) = Rotation(row, col);
        }
    }
    ownPose(0, 3) = translation(0, 0);
    ownPose(1, 3) = translation(1, 0);
    ownPose(2, 3) = translation(2, 0);

    T = GT * ownPose.inverse();

}

