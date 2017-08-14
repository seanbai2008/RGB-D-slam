/*************************************************************************
     > File Name: rgbd-slam-tutorial-gx/part III/code/include/slamBase.h
     > Author: xiang gao
     > Mail: gaoxiang12@mails.tsinghua.edu.cn
     > Created Time: 2015年07月18日 星期六 15时14分22秒
     > 说明：rgbd-slam教程所用到的基本函数（C风格）
  ************************************************************************/
 // # pragma once

 // 各种头文件
 // C++标准库
#include <fstream>
#include <vector>
using namespace std;

// OpenCV

// #include "opencv2/xfeatures2d.hpp"
// #include <opencv2/features2d/features2d.hpp>
// #include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "ros/time.h"

#include "ros/ros.h"

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
   double cx, cy, fx, fy, scale;
};

 // 帧结构
struct FRAME
{
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
    int frameID;
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME};

// 参数读取类
class ParameterReader
{
  public:
      ParameterReader( string filename="/home/fan/test_dir/develop/src/RGB-D/src/data/parameters.txt" )
      {
          ifstream fin( filename.c_str() );
          if (!fin)
          {
              cerr<<"parameter file does not exist."<<endl;
              return;
          }
          while(!fin.eof())
          {
              string str;
              getline( fin, str );
              if (str[0] == '#')
              {
                  // 以‘＃’开头的是注释
                  continue;
              }

              int pos = str.find("=");
              if (pos == -1)
                  continue;
              string key = str.substr( 0, pos );
              string value = str.substr( pos+1, str.length() );
              data[key] = value;

              if ( !fin.good() )
                  break;
          }
      }
      string getData( string key )
      {
          map<string, string>::iterator iter = data.find(key);
          if (iter == data.end())
          {
              cerr<<"Parameter name "<<key<<" not found!"<<endl;
              return string("NOT_FOUND");
          }
          return iter->second;
      }
  public:
      map<string, string> data;
};

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp( FRAME& frame);

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion( const FRAME& frame1,  const FRAME& frame2, const CAMERA_INTRINSIC_PARAMETERS& camera );

// 函数接口
// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud(  const cv::Mat& rgb, const cv::Mat& depth, const CAMERA_INTRINSIC_PARAMETERS& camera);

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, const CAMERA_INTRINSIC_PARAMETERS& camera );

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

// joinPointCloud
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera );

void EulerToOdometry(cv::Mat& rvec, cv::Mat& tvec , Eigen::Isometry3d& Odom);

nav_msgs::Odometry OdometryToRos(Eigen::Isometry3d& Odom);

CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops, CAMERA_INTRINSIC_PARAMETERS& camera);

void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti , CAMERA_INTRINSIC_PARAMETERS& camera);

void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti, CAMERA_INTRINSIC_PARAMETERS& camera );

double normofTransform( cv::Mat rvec, cv::Mat tvec );
