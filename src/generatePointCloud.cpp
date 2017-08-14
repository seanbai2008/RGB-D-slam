// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

 // PCL 库
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <pcl/filters/statistical_outlier_removal.h>

 // 定义点云类型
 typedef pcl::PointXYZRGBA PointT;
 typedef pcl::PointCloud<PointT> PointCloud;

 // 相机内参
 const double camera_factor = 1000;
 const double camera_cx = 325.5;
 const double camera_cy = 253.5;
 const double camera_fx = 518.0;
 const double camera_fy = 519.0;

 // const double camera_cx = 971.96923828125;
 // const double camera_cy = 593.4072875976562 ;
 // const double camera_fx = 1388.6114501953125;
 // const double camera_fy = 1388.6114501953125;

 // 主函数
 int main( int argc, char** argv )
 {
     //get depth and rgb information
     cv::Mat rgb, depth;
     rgb = cv::imread( "/home/fan/test_dir/develop/src/RGB-D/src/data/rgb3.png" );
     depth = cv::imread( "/home/fan/test_dir/develop/src/RGB-D/src/data/depth3.png", -1 );

     //create point cloud
     PointCloud::Ptr cloud ( new PointCloud );
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

     //combine rgb with depth to create a 3d point cloud
     for (int m = 0; m < depth.rows; m++){
         for (int n=0; n < depth.cols; n++)
         {

             ushort d = depth.ptr<ushort>(m)[n];

             if (d == 0)
                 continue;

             PointT p;

             p.z = double(d) / camera_factor;
             p.x = (n - camera_cx) * p.z / camera_fx;
             p.y = (m - camera_cy) * p.z / camera_fy;

             p.b = rgb.ptr<uchar>(m)[n*3];
             p.g = rgb.ptr<uchar>(m)[n*3+1];
             p.r = rgb.ptr<uchar>(m)[n*3+2];

             cloud->points.push_back( p );
         }
       }

     // filter the point cloud
     pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
     sor.setInputCloud (cloud);
     sor.setMeanK (50);
     sor.setStddevMulThresh (1.0);
     sor.filter (*cloud_filtered);

     cloud_filtered->height = 1;
     cloud_filtered->width = cloud_filtered->points.size();
     cout<<"point cloud size = "<<cloud_filtered->points.size()<<endl;

     //True if no points are invalid
     cloud_filtered->is_dense = false;
     pcl::io::savePCDFile( "/home/fan/test_dir/develop/src/RGB-D/pointcloud.pcd", *cloud_filtered );

     cloud_filtered->points.clear();
     cout<<"Point cloud saved."<<endl;
     return 0;
 }
