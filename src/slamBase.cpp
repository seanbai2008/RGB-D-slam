/*************************************************************************
    > File Name: src/slamBase.cpp
    > Author: xiang gao
    > Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Implementation of slamBase.h
    > Created Time: 2015年07月18日 星期六 15时31分49秒
 ************************************************************************/

#include "slamBase.h"

cv::Ptr<cv::FeatureDetector> _detector = cv::FeatureDetector::create( "ORB" );
cv::Ptr<cv::DescriptorExtractor> _descriptor = cv::DescriptorExtractor::create( "ORB") ;


// cv::initModule_nonfree();

 PointCloud::Ptr image2PointCloud( const cv::Mat& rgb, const cv::Mat& depth, const CAMERA_INTRINSIC_PARAMETERS& camera )
 {
     PointCloud::Ptr cloud ( new PointCloud );

     for (int m = 0; m < depth.rows; m++)
         for (int n=0; n < depth.cols; n++)
         {
             // 获取深度图中(m,n)处的值
             float d = depth.ptr<float>(m)[n];

            //  cout<<m<<" "<<n<<" "<<d<<endl;
             // d 可能没有值，若如此，跳过此点
             if (d == 0 || d!=d)
                 continue;
             // d 存在值，则向点云增加一个点

             PointT p;

             // 计算这个点的空间坐标
             p.z = double(d) ;
             p.x = (n - camera.cx) * p.z / camera.fx;
             p.y = (m - camera.cy) * p.z / camera.fy;



             // 从rgb图像中获取它的颜色
             // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
             p.b = rgb.ptr<uchar>(m)[n*3];
             p.g = rgb.ptr<uchar>(m)[n*3+1];
             p.r = rgb.ptr<uchar>(m)[n*3+2];
            //  cout<<p.b<<p.g<<p.r<<endl;
             // 把p加入到点云中
             cloud->points.push_back( p );
            //  cout<<p<<endl;
         }

     // 设置并保存点云
     cloud->height = 1;
     cloud->width = cloud->points.size();
     cloud->is_dense = false;

     return cloud;
 }

 cv::Point3f point2dTo3d( cv::Point3f& point, const CAMERA_INTRINSIC_PARAMETERS& camera ){
     cv::Point3f p; // 3D 点
     p.z = double( point.z);
     p.x = ( point.x - camera.cx) * p.z / camera.fx;
     p.y = ( point.y - camera.cy) * p.z / camera.fy;

    //  p.y = double( point.z);
    //  p.x = ( point.x - camera.cx) * p.y / camera.fx;
    //  p.z = ( point.y - camera.cy) * p.y / camera.fy;

     return p;
}

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp( FRAME& frame)
{


    // if (!_detector || !_descriptor)
    // {
    //     cerr<<"Unknown detector or discriptor type !"<<detector<<","<<descriptor<<endl;
    //     return;
    // cout<<"debug ..."<<endl;
    _detector->detect( frame.rgb, frame.kp );
    _descriptor->compute( frame.rgb, frame.kp, frame.desp );

    return;
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
RESULT_OF_PNP estimateMotion( const FRAME& frame1, const FRAME& frame2, const CAMERA_INTRINSIC_PARAMETERS& camera )
{
    static ParameterReader pd;

    // if(frame1.desp.type()!=CV_32F) {
    // frame1.desp.convertTo(frame1.desp, CV_32F);
    // }
    //
    // if(frame2.desp.type()!=CV_32F) {
    //     frame2.desp.convertTo(frame2.desp, CV_32F);
    // }

    vector< cv::DMatch > matches;

    // cv::Ptr< cv::flann::IndexParams > indexParams = new cv::flann::KDTreeIndexParams(4);
    // cv::Ptr< cv::flann::SearchParams >  searchParams = new cv::flann::SearchParams(100,0,true);
    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(5, 24, 2));
    matcher.match( frame1.desp, frame2.desp, matches );

    cout<<"find total "<<matches.size()<<" matches."<<endl;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    if(minDis==0) minDis =5;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis)
            goodMatches.push_back( matches[i] );
    }

    cout<<"good matches: "<<goodMatches.size()<<endl;
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        float d = frame1.depth.ptr<float>( int(p.y) )[ int(p.x) ];

        if (d < 0.1 || d!=d )
            continue;

        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }


    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // cout<<"solving pnp"<<endl;
    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    try {
      cout<<"PNP"<<endl;
      cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 500, 0.6, 100, inliers );
      // cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 1000, 0.6, 0.99, inliers,cv::SOLVEPNP_ITERATIVE );
      cout<<"PNP finished"<<endl;
     }
     catch( cv::Exception& e )
     {
        //  const char* err_msg = e.what();
        //  std::cout << "exception caught: " << err_msg << std::endl;
     }

    RESULT_OF_PNP result;
    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    cout<<result.inliers<<endl;
    return result;
}

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(0,1);
    T(2,3) = tvec.at<double>(0,2);
    return T;
}

// joinPointCloud
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
}

void EulerToOdometry(cv::Mat& rvec, cv::Mat& tvec, Eigen::Isometry3d& Odom){

  Eigen::Isometry3d T = cvMat2Eigen( rvec,tvec );
  Odom = Odom * T;

  return;
}

nav_msgs::Odometry OdometryToRos(Eigen::Isometry3d& T){

  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "visual_odom";
  //set the position
  msg.pose.pose.position.x = T(0,3);
  msg.pose.pose.position.y = T(1,3);
  msg.pose.pose.position.z = T(2,3);

  Eigen::Quaterniond odom_quat(T.rotation());

  msg.pose.pose.orientation.x = odom_quat.coeffs()[0];
  msg.pose.pose.orientation.y = odom_quat.coeffs()[1];
  msg.pose.pose.orientation.z = odom_quat.coeffs()[2];
  msg.pose.pose.orientation.w = odom_quat.coeffs()[3];

  //set the velocity
  msg.child_frame_id = "camera_link";
  msg.twist.twist.linear.x = 0;
  msg.twist.twist.linear.y = 0;
  msg.twist.twist.angular.z = 0;

  return msg;

}

CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops , CAMERA_INTRINSIC_PARAMETERS& camera)
{
    static ParameterReader pd;
    static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );


    // static CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );
    // 比较f1 和 f2
    RESULT_OF_PNP result = estimateMotion( f1, f2, camera );
    if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
        return NOT_MATCHED;
    // 计算运动范围是否太大
    double norm = normofTransform(result.rvec, result.tvec);
    if ( is_loops == false )
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;   // too adjacent frame
    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点部分
    // 顶点只需设定id即可
    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( f2.frameID );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
    // 边部分
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 连接此边的两个顶点id
    edge->vertices() [0] = opti.vertex( f1.frameID );
    edge->vertices() [1] = opti.vertex( f2.frameID );
    edge->setRobustKernel( robustKernel );
    // 信息矩阵
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation( information );
    // 边的估计即是pnp求解之结果
    Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
    edge->setMeasurement( T.inverse() );
    // 将此边加入图中
    opti.addEdge(edge);
    return KEYFRAME;
}

void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti ,CAMERA_INTRINSIC_PARAMETERS& camera )
{
    static ParameterReader pd;
    static int nearby_loops = atoi( pd.getData("nearby_loops").c_str() );

    // 就是把currFrame和 frames里末尾几个测一遍
    if ( frames.size() <= nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true, camera );
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true, camera );
        }
    }
}

void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti ,CAMERA_INTRINSIC_PARAMETERS& camera)
{
    static ParameterReader pd;
    static int random_loops = atoi( pd.getData("random_loops").c_str() );
    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测

    if ( frames.size() <= random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true , camera );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes( frames[index], currFrame, opti, true ,camera );
        }
    }
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}
