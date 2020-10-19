#include "pointCloudMapping.hpp"

void showPointCloud(
    const vector<VectorXd, Eigen::aligned_allocator<VectorXd>> &pointcloud);


void pointCloudMapping::insertValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud, Mat&depthmap, Mat&image1, Matrix4f&TransformsValue, vector<float>&camera, float scale, vector<vector<float> > Centerpoint, Mat Labels){
    //cout<<"received the depth map, start generate the point cloud"<<endl;
    image = image1;
    resize(image,image,Size(),scale,scale);
    depth = depthmap;

    //cout<<depth<<endl;
    s = scale;
    baseline =  camera[0]*s;
    fx = camera[0]*s;
    fy = camera[1]*s; 
    cx = camera[2]*s; 
    cy = camera[3]*s;
    Cpoint = Centerpoint;
    LabelValue = Labels;
    Cloud = pcl::make_shared<pcl::PointCloud<PointXYZRGB>>( );
    GlobalCloud = pcl::make_shared<pcl::PointCloud<PointXYZRGB>>( );
    *GlobalCloud = *insertCloud;
    //cout<<"global size: "<<GlobalCloud->points.size()<<endl;
    T = TransformsValue;
    cout<<"T: "<<T<<endl;
}

void pointCloudMapping::initialize3Dmap(int type){
    // cloud->width = depth.cols*depth.rows;
    // cloud->height = 1;
    // cloud->points.resize (cloud->width * cloud->height);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>); 
    cout<<"type: "<<depth.type()<<endl;
    for(int i=0;i<depth.rows;i++) { 
            if(i%4 != 0){
                continue;
            }

        for(int j =0; j<depth.cols; j++){
            if(j%4 != 0){
                continue;
            }
            pcl::PointXYZRGB point;

            float u0= j;
            float v0= i;
            float du0=u0-cx;
            float dv0=v0-cy;
            float x0=du0/fx;
            float y0=dv0/fx;


            float Dep;
            Dep = (float(depth.at<uchar>(i, j))/255)*100;
            if(Dep > 50){
                continue;
            }

            //cout<<Dep<<endl;
            float X=x0* Dep;
            float Y=y0 * Dep;
            float Z=Dep;
            
            int number = i*depth.cols+j;
            point.x = Z;
            point.y = X;
            point.z = Y;

            int r, g, b;
            b =  image.at<Vec3b>(i, j)[0];
            g=  image.at<Vec3b>(i, j)[1];
            r =  image.at<Vec3b>(i, j)[2];


            unsigned char R= r;
            unsigned char G= g;
            unsigned char B= b;

            uint8_t rp = r;  
            uint8_t gp = g; 
            uint8_t bp = b;
            uint32_t rgb = ((uint32_t )rp<<16 | (uint32_t )gp<<8 |  (uint32_t )bp);
            point.rgb = *reinterpret_cast<float*>(&rgb);

            temp->push_back(point);
        }
    }
    transformPointCloud (*temp, *temp, T);
    //cout<<test<<endl;
    *Cloud = *temp;
    cout<<"The original point cloud has: "<<Cloud->points.size()<<" points"<<endl;
}

void Transformation(float& averageX3, float& averageY3,float& averageZ3, Matrix4f T){
    
    Eigen::Vector4f point; 
    point[0] = averageX3; 
    point[1] = averageY3;
    point[2] = averageZ3; 
    point[3] = 1;
    Eigen::Vector4f pointWorld = T*point;
    averageX3 = pointWorld[0];
    averageY3 = pointWorld[1];
    averageZ3 = pointWorld[2];

}

void Seed_Filling(const cv::Mat& binImg, cv::Mat& lableImg, cv::Mat& image, cv::Mat& depth, vector<int>& centerX, vector<int>& centerY, 
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp, vector<vector<float> >& Cpoint, int lab, Mat LabelValue, Matrix4f T)  
{
 
 
	if (binImg.empty() ||
		binImg.type() != CV_8UC1)
	{
		return;
	}
    
	lableImg.release();
	binImg.convertTo(lableImg, CV_32SC1);

    //cout<<lableImg.type()<<endl;
    Mat imgClone = Mat(lableImg.rows + 1, lableImg.cols + 1, lableImg.type(), Scalar(0));
    lableImg.copyTo(imgClone(Rect(1, 1, lableImg.cols, lableImg.rows)));

    Mat depthClone = Mat(depth.rows + 1, depth.cols + 1, depth.type(), Scalar(0));
    depth.copyTo(depthClone(Rect(1, 1, depth.cols, depth.rows)));
    
	int label = 20;  
 
	int rows = imgClone.rows - 1;  
	int cols = imgClone .cols - 1;
	for (int i = 1; i < rows-1; i++)
	{
		int* data= imgClone.ptr<int>(i);
		for (int j = 1; j < cols-1; j++)
		{
			if (data[j] == 255)
			{
				std::stack<std::pair<int,int>> neighborPixels;   
				neighborPixels.push(std::pair<int,int>(i,j)); 
				label = label + 1; 
                if(label == 255){
                    label = label+1;
                }
                int count = 0;
                float sumX = 0;
                float sumY = 0;
                float sum3dX = 0;
                float sum3dY = 0;
                float sum3dZ = 0;
                // cout<<"j: "<<j<<endl;
				while (!neighborPixels.empty())
				{
					std::pair<int,int> curPixel = neighborPixels.top(); 
					int curX = curPixel.first;
					int curY = curPixel.second;
					imgClone.at<int>(curX, curY) = label;
 
					neighborPixels.pop();
                    float Dep0, Dep1, Dep2, Dep3, Dep4;
                    Dep0 = (float(depthClone.at<uchar>(curX, curY))/255)*100;
                    Dep1 = (float(depthClone.at<uchar>(curX, curY-1))/255)*100;
                    Dep2 = (float(depthClone.at<uchar>(curX, curY+1))/255)*100;
                    Dep3 = (float(depthClone.at<uchar>(curX-1, curY))/255)*100;
                    Dep4 = (float(depthClone.at<uchar>(curX+1, curY))/255)*100;

					if (imgClone.at<int>(curX, curY-1) == 255 && abs(Dep0 - Dep1)<1)
					{
						neighborPixels.push(std::pair<int,int>(curX, curY-1));
					}
					if (imgClone.at<int>(curX, curY+1) == 255 && abs(Dep0 - Dep2)<1)
					{
						neighborPixels.push(std::pair<int,int>(curX, curY+1));
					}
					if (imgClone.at<int>(curX-1, curY) == 255 && abs(Dep0 - Dep3)<1)
					{
						neighborPixels.push(std::pair<int,int>(curX-1, curY));
					}
					if (imgClone.at<int>(curX+1, curY) == 255 && abs(Dep0 - Dep4)<1)
					{
						neighborPixels.push(std::pair<int,int>(curX+1, curY));
					}
                    // if (imgClone.at<int>(curX+1, curY+1) == 255)
					// {
					// 	neighborPixels.push(std::pair<int,int>(curX+1, curY+1));
					// }
                    // if (imgClone.at<int>(curX+1, curY-1) == 255)
					// {
					// 	neighborPixels.push(std::pair<int,int>(curX+1, curY-1));
					// }
                    // if (imgClone.at<int>(curX-1, curY-1) == 255)
					// {
					// 	neighborPixels.push(std::pair<int,int>(curX-1, curY-1));
					// }
                    // if (imgClone.at<int>(curX-1, curY+1) == 255)
					// {
					// 	neighborPixels.push(std::pair<int,int>(curX-1, curY+1));
					// }

                    //generate 2D center
                    float u0 = (float)curY;
                    float v0 = (float)curX;
                    u0 = u0-1;
                    v0 = v0-1;
                    sumX = sumX + u0;
                    sumY = sumY + v0;
                    // if(count >8070 && count<8100){
                    //     cout<<"count: "<<count<<endl;
                    //     cout<<"Y: "<<curY<<endl;
                    // }

                    float du0=u0-512;
                    float dv0=v0-288;
                    float x0=du0/512;
                    float y0=dv0/512;

                    float Dep;
                    Dep = (float(depthClone.at<uchar>(curX, curY))/255)*100;
                    if(Dep > 30){
                        continue;
                    }
                    count = count + 1;
                    //cout<<Dep<<endl;
                    float X=x0* Dep;
                    float Y=y0 * Dep;
                    float Z=Dep;
                    sum3dX = sum3dX + X;
                    sum3dY = sum3dY + Y;
                    sum3dZ = sum3dZ + Z;

				}
                if(count >5000){
                    float averageXf = sumX / count;
                    float averageYf = sumY / count;
                    int aveX = (int)averageXf;
                    int aveY = (int)averageYf;
                    //cout<<"aveX: "<<aveX<<endl;
                    //cout<<"aveY: "<<aveY<<endl;
                    centerX.push_back(aveX);
                    centerY.push_back(aveY);
                    //point
                    float averageX3 = sum3dZ / count;
                    float averageY3 = sum3dX / count;
                    float averageZ3 = sum3dY / count;
                    pcl::PointXYZRGB point;
                    point.x = averageX3;
                    point.y = averageY3;
                    point.z = averageZ3;
                    //cout<<"Z: "<<point.z<<endl;
                    int b =  LabelValue.at<int>(lab, 2);
                    int g=  LabelValue.at<int>(lab, 1);
                    int r =  LabelValue.at<int>(lab, 0);

                    unsigned char R= r;
                    unsigned char G= g;
                    unsigned char B= b;

                    uint8_t rp = r;  
                    uint8_t gp = g; 
                    uint8_t bp = b;
                    uint32_t rgb = ((uint32_t )rp<<16 | (uint32_t )gp<<8 |  (uint32_t )bp);
                    point.rgb = *reinterpret_cast<float*>(&rgb);

                    vector<float> cp(4);
                    Transformation(averageX3, averageY3, averageZ3, T);
                    cp[0] = averageX3; cp[1] = averageY3; cp[2] = averageZ3; cp[3] = (float)lab;
                    int CSzie = Cpoint.size();
                    //cout<<"Cpoint size: "<<CSzie<<endl;
                    if(CSzie>0){
                        bool fuse = false;
                        float Distance = 10000;
                        for(int sizeI = 0; sizeI<CSzie; sizeI++){
                            float previousLab = Cpoint[sizeI][3];
                            if(cp[3] == previousLab){
                                Distance = sqrt(pow(Cpoint[sizeI][0] - cp[0],2) + pow(Cpoint[sizeI][1] - cp[1],2) + pow(Cpoint[sizeI][2] - cp[2],2));
                                //cout<<"distance: "<<Distance<<endl;
                                if(Distance<5 && lab>7){
                                    // cout<<Cpoint[sizeI][2]<<endl;
                                    // cout<<cp[2]<<endl;
                                    //cout<<"______________Road point fuse________________"<<endl;
                                    fuse = true;
                                }
                                else if(Distance<5 && lab<8){
                                    // Cpoint[sizeI][0] = (Cpoint[sizeI][0] + cp[0])/2;
                                    // Cpoint[sizeI][1] = (Cpoint[sizeI][1] + cp[1])/2;
                                    // Cpoint[sizeI][2] = (Cpoint[sizeI][2] + cp[2])/2;
                                    //cout<<"______________Object point fuse________________"<<endl;
                                    fuse = true;
                                }
                            }
                        }
                        if(fuse == true){
                            continue;
                        }
                    }
                    temp->push_back(point);
                    Cpoint.push_back(cp);

                }		
			}
		}
	}
	
}

void pointCloudMapping::pointExtraction(vector<uchar> label){

    Mat image_gray;
    vector<int> centerX;
    vector<int> centerY;
    cvtColor(image, image_gray, CV_BGR2GRAY);
    int imgRow = image.rows; int imgCol = image.cols;
    //cout<<image_gray.row(270)<<endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>); 
    for(int lab = 0; lab<11; lab++){
        Mat image_bw = cv::Mat::zeros(imgRow, imgCol, CV_8UC1);
        for(int row = 0; row<imgRow; row++){
            for(int col = 0; col<imgCol; col++){
                if(image_gray.at<uchar>(row, col) == label[lab]){
                    image_bw.at<uchar>(row, col) = 255;
                }
                else{
                    image_bw.at<uchar>(row, col) = 0;
                }

            }
        }
        cv::Mat labelImg;
        // /cout<<"label: "<<lab<<endl;
        Seed_Filling(image_bw, labelImg, image, depth, centerX, centerY, temp, Cpoint, lab, LabelValue, T);
        //cout<<centerX.size()<<endl;
        for(int sizeNum = 0; sizeNum<centerX.size(); sizeNum++){
            Point p1;
            p1.x = centerX[sizeNum];
            p1.y = centerY[sizeNum];
            circle(image, p1, 15,Scalar(255,255,0),3);
        }
        // imshow("bbb", image_bw);
        // waitKey(2000);

    }
    imshow("aaa", image);
    transformPointCloud (*temp, *temp, T);
    *Cloud = *temp;
    cout<<"The number of keypoint is: "<<Cloud->points.size()<<" points"<<endl;    
}

void pointCloudMapping::pointExtraction(vector<uchar> label, Mat labelimag){
    cout<<labelimag.type()<<endl;
    vector<int> centerX;
    vector<int> centerY;
    int imgRow = image.rows; int imgCol = image.cols;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>); 
    for(int lab = 0; lab<9; lab++){
        Mat image_bw = cv::Mat::zeros(imgRow, imgCol, CV_8UC1);
        for(int row = 0; row<imgRow; row++){
            for(int col = 0; col<imgCol; col++){
                if(labelimag.at<uchar>(row, col) == label[lab]){
                    image_bw.at<uchar>(row, col) = 255;
                }
                else{
                    image_bw.at<uchar>(row, col) = 0;
                }
            }
        }
        cv::Mat labelImg;
        // /cout<<"label: "<<lab<<endl;
        Seed_Filling(image_bw, labelImg, image, depth, centerX, centerY, temp, Cpoint, lab, LabelValue, T);
        //cout<<centerX.size()<<endl;
        for(int sizeNum = 0; sizeNum<centerX.size(); sizeNum++){
            Point p1;
            p1.x = centerX[sizeNum];
            p1.y = centerY[sizeNum];
            circle(image, p1, 15,Scalar(255,255,0),3);
        }
        // imshow("bbb", image_bw);
        // waitKey(2000);

    }
    imshow("aaa", image);
    transformPointCloud (*temp, *temp, T);
    *Cloud = *temp;
    cout<<"The number of keypoint is: "<<Cloud->points.size()<<" points"<<endl;    
}

void pointCloudMapping::pointCloudFilter(){

    pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  
    downSampled.setInputCloud (Cloud);            
    downSampled.setLeafSize (1.8f, 1.8f, 1.8f);  
    downSampled.setDownsampleAllData(true);
    downSampled.filter (*Cloud);
              
    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> pcFilter;  
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);    
    // pcFilter.setInputCloud(Cloud);             
    // pcFilter.setRadiusSearch(200);        
    // pcFilter.setMinNeighborsInRadius(4);      
    // pcFilter.filter(*Cloud);       

    cout<<"The filtered point cloud has: "<<Cloud->points.size()<<" points"<<endl;
        
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudMapping::outputPointCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>); 
    *temp = *GlobalCloud+*Cloud;
    return temp;
}

void showPointCloud(const vector<VectorXd, Eigen::aligned_allocator<VectorXd>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }
    //创建一个窗口
    pangolin::CreateWindowAndBind("Point Cloud Viewer_1", 1024, 768);
    //启动深度测试
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::DataLog log;
    std::vector<std::string> labels;
    labels.push_back(std::string("sin(t)"));
    log.SetLabels(labels);
    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        // Clear screen and active view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        pangolin::glDrawAxis(3);
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        glPointSize(20);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[4], p[5]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        // Swap frames and Process Events
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

void pointCloudMapping::pointVisuallize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud2, MatrixXi matcherID, MatrixXf R, MatrixXf T){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZRGB>);
    //obtain the clouds.
    Matrix4f TransforMatrix = Matrix4f::Identity();
    // for(int row = 0; row<3; row++){
    //     for(int col=0; col<3; col++){
    //         TransforMatrix(row, col) = R(row, col);
    //     }
    // }
    // TransforMatrix(0, 3) = T(0, 0);
    // TransforMatrix(1, 3) = T(1, 0);
    // TransforMatrix(2, 3) = T(2, 0);

    TransforMatrix(2, 3) = TransforMatrix(2, 3)+80;  
    *temp1 = *insertCloud1;
    *temp2 = *insertCloud2;
    transformPointCloud (*temp1, *temp1, TransforMatrix);
    *temp = *temp1 + *temp2;
    //obtain the line information
    int size = matcherID.rows();

    //cout<<" has: "<<temp->points.size()<<" points"<<endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point cloud"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(temp);
    viewer->addPointCloud<pcl::PointXYZRGB>(temp, rgb, "Point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "Point cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);

    int count = 0;
    int line_numeber = 0;
    char str[25];//
    while(!viewer->wasStopped()){
        viewer->spinOnce(1000);
        line_numeber++;
        sprintf(str, "%d", line_numeber);
        if(count < size){
            viewer->addLine<pcl::PointXYZRGB> (temp1->points[matcherID(count, 0)], temp2->points[matcherID(count, 1)], str);
        }
        count++;

    }

    // vector<VectorXd, Eigen::aligned_allocator<VectorXd>> pointcloud;
    // cout<<"global size: "<<insertCloud->points.size()<<endl;
    // for(int i = 0; i<insertCloud->points.size(); i++){
    //         int r = insertCloud->points[i].r;
    //         int g = insertCloud->points[i].g;
    //         int b = insertCloud->points[i].b;
    //         Eigen::VectorXd point1(6);
    //         point1<< 0, 0, 0, r/ 255.0, g/255.0, b/255.0;
    //         point1[0] = double(insertCloud->points[i].x);
    //         //cout << "X" << " " << point1[0]  << endl;
    //         point1[1] = double(insertCloud->points[i].y);
    //         //cout << "Y" << " " << point1[1] << endl;
    //         point1[2] = double(insertCloud->points[i].z);
    //         //cout << "Z" << " " << point1[2] << endl;
    //         pointcloud.push_back(point1);
    // }
    // showPointCloud(pointcloud);
}