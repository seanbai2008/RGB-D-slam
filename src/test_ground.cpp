#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

// #include "opencv2/xfeatures2d.hpp"
// #include <opencv2/features2d/features2d.hpp>
// #include <opencv2/calib3d/calib3d.hpp>



#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "slamBase.h"

using namespace std;

typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

// const double camera_factor = 1000;
// const double camera_cx = 645.9493408203125;
// const double camera_cy = 385.9999084472656 ;
// const double camera_fx = 680.7481079101562;
// const double camera_fy = 680.7481079101562;

// D: [0.0, 0.0, 0.0, 0.0, 0.0]
// K: [570.3422241210938, 0.0, 319.5, 0.0, 570.3422241210938, 239.5, 0.0, 0.0, 1.0]
// R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
// P: [570.3422241210938, 0.0, 319.5, 0.0, 0.0, 570.3422241210938, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]


CAMERA_INTRINSIC_PARAMETERS camera;
ParameterReader pd;

static cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( "ORB" );
static cv::Ptr<cv::DescriptorExtractor> descriptor = cv:: DescriptorExtractor::create( "ORB" );

static Eigen::Isometry3d Odom = Eigen::Isometry3d::Identity();
static  ros::Publisher odom_pub;

static g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
static SlamLinearSolver* linearSolver;
static SlamBlockSolver* blockSolver;
static g2o::OptimizationAlgorithmLevenberg* solver;

static vector< FRAME > keyframes;

bool visualize = pd.getData("visualize_pointcloud")==string("yes");
int min_inliers = atoi( pd.getData("min_inliers").c_str() );
double max_norm = atof( pd.getData("max_norm").c_str() );



FRAME readFrame( const sensor_msgs::Image::ConstPtr& rgb , const sensor_msgs::Image::ConstPtr& depth , int& frame_count);

double normofTransform( cv::Mat rvec, cv::Mat tvec );



// static PointCloud::Ptr cloud;
// static pcl::visualization::CloudViewer viewer("viewer");

void callback(const sensor_msgs::Image::ConstPtr& rgb , const sensor_msgs::Image::ConstPtr& depth){
  static FRAME currFrame, lastFrame;
  static int frame_count = 0;
  static RESULT_OF_PNP result;
  static g2o::VertexSE3* v;

  switch(frame_count){
    case 0:

      lastFrame = readFrame(rgb,depth, frame_count);
      computeKeyPointsAndDesp(lastFrame);


      globalOptimizer.setAlgorithm( solver );
      globalOptimizer.setVerbose( false );

      // 向globalOptimizer增加第一个顶点
      v = new g2o::VertexSE3();
      v->setId( lastFrame.frameID );
      v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
      v->setFixed( false ); //第一个顶点固定，不用优化
      globalOptimizer.addVertex( v );


      keyframes.push_back( lastFrame );

      // cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );
      // viewer.showCloud( cloud );
      break;

    default:
      //get new msg
      currFrame = readFrame(rgb,depth ,frame_count);
      computeKeyPointsAndDesp(currFrame);

      // result = estimateMotion(lastFrame,currFrame,camera);
      CHECK_RESULT PnP_result = checkKeyframes( keyframes.back(), currFrame, globalOptimizer, false, camera ); //匹配该帧与keyframes里最后一帧


      //if no odom is updated, still publish the old one
      odom_pub.publish(OdometryToRos(Odom));

      switch (PnP_result){
      case NOT_MATCHED:
          //没匹配上，直接跳过
          cout<<"Not enough inliers."<<endl;
          break;
      case TOO_FAR_AWAY:
          // 太近了，也直接跳
          cout<<"Too far away, may be an error."<<endl;
          break;
      case TOO_CLOSE:
          // 太远了，可能出错了
          cout<<"Too close, not a keyframe"<<endl;
          break;
      case KEYFRAME:
          cout<<"This is a new keyframe"<<endl;

          checkNearbyLoops( keyframes, currFrame, globalOptimizer, camera);
          checkRandomLoops( keyframes, currFrame, globalOptimizer, camera);
          // }
          keyframes.push_back( currFrame );

          globalOptimizer.initializeOptimization();
          globalOptimizer.optimize( 10 ); //可以指定优化步数

          g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( (keyframes.back()).frameID ));
          Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿


          cout<<"Odom: "<<pose.matrix()<<endl;
          Odom = pose;
          // EulerToOdometry(result.rvec, result.tvec, Odom);
          //
          // Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );

          // lastFrame = currFrame;

          break;
      }
      break;
  }
}

int main( int argc, char** argv ){

  cout<<"Initializing ..."<<endl;

  ros::init(argc, argv, "talker");

  camera.fx = atof( pd.getData( "camera.fx" ).c_str());
  camera.fy = atof( pd.getData( "camera.fy" ).c_str());
  camera.cx = atof( pd.getData( "camera.cx" ).c_str());
  camera.cy = atof( pd.getData( "camera.cy" ).c_str());
  camera.scale = atof( pd.getData( "camera.scale" ).c_str() );


  linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering( false );
  blockSolver = new SlamBlockSolver(linearSolver);
  solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver);
  globalOptimizer.setAlgorithm( solver );
  globalOptimizer.setVerbose( false );

  ros::NodeHandle nh;

  odom_pub = nh.advertise<nav_msgs::Odometry>("/visual_odom", 100);

  // initailize the callback function for taking the depth and image information from ZED
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_rect_color", 10);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect", 10);
  // message_filters::Subscriber<sensor_msgs::CameraInfo> came_sub(nh, "/camera/depth/camera_info", 10);
  // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(rgb_sub, depth_sub,10);

  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> > sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(10), rgb_sub, depth_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}

//read ROS message function, get rgb image and depth information from camera
FRAME readFrame( const sensor_msgs::Image::ConstPtr& rgb , const sensor_msgs::Image::ConstPtr& depth , int& frame_count){

  //here is to obtain the rgb and depth info from Ros message
  FRAME frame;

  cv_bridge::CvImageConstPtr cv_ptr1;

  //what is the difference between cvtocopy and cvtoshare?
  cv_ptr1 = cv_bridge::toCvCopy(rgb, "bgr8");

  frame.rgb = cv_ptr1->image;
  cv_bridge::CvImageConstPtr cv_ptr2;
  cv_ptr2 = cv_bridge::toCvCopy(depth,depth->encoding);

  frame.depth = cv_ptr2->image;

  frame.frameID = frame_count;

  frame_count++;

  return frame;
}
