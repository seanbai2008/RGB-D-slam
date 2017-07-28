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
 // const double camera_cx = 325.5;
 // const double camera_cy = 253.5;
 // const double camera_fx = 518.0;
 // const double camera_fy = 519.0;

 const double camera_cx = 971.96923828125;
 const double camera_cy = 593.4072875976562 ;
 const double camera_fx = 1388.6114501953125;
 const double camera_fy = 1388.6114501953125;

 // 主函数
 int main( int argc, char** argv )
 {
     // 读取./data/rgb.png和./data/depth.png，并转化为点云

     // 图像矩阵
     cv::Mat rgb, depth;
     // 使用cv::imread()来读取图像
     // API: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html?highlight=imread#cv2.imread
     rgb = cv::imread( "/home/fan/test_dir/develop/src/RGB-D/src/data/rgb3.png" );
     // rgb 图像是8UC3的彩色图像
     // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
     depth = cv::imread( "/home/fan/test_dir/develop/src/RGB-D/src/data/depth3.png", -1 );

     // 点云变量
     // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
     PointCloud::Ptr cloud ( new PointCloud );
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

     // 遍历深度图
     cout<<depth.size()<<endl;

     for (int m = 0; m < depth.rows; m++){
         for (int n=0; n < depth.cols; n++)
         {

             // 获取深度图中(m,n)处的值
             ushort d = depth.ptr<ushort>(m)[n];


             // d 可能没有值，若如此，跳过此点
             if (d == 0)
                 continue;
             // d 存在值，则向点云增加一个点

             PointT p;

             // 计算这个点的空间坐标
             p.z = double(d) / camera_factor;
             p.x = (n - camera_cx) * p.z / camera_fx;
             p.y = (m - camera_cy) * p.z / camera_fy;

             // 从rgb图像中获取它的颜色
             // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
             p.b = rgb.ptr<uchar>(m)[n*3];
             p.g = rgb.ptr<uchar>(m)[n*3+1];
             p.r = rgb.ptr<uchar>(m)[n*3+2];
             // 把p加入到点云中
             cloud->points.push_back( p );
         }
       }


     pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
     sor.setInputCloud (cloud);
     sor.setMeanK (50);
     sor.setStddevMulThresh (1.0);
     sor.filter (*cloud_filtered);
     // 设置并保存点云
     cloud_filtered->height = 1;
     cloud_filtered->width = cloud_filtered->points.size();
     cout<<"point cloud size = "<<cloud_filtered->points.size()<<endl;

     //True if no points are invalid
     cloud_filtered->is_dense = false;
     pcl::io::savePCDFile( "/home/fan/test_dir/develop/src/RGB-D/pointcloud.pcd", *cloud_filtered );
     // 清除数据并退出
     cloud_filtered->points.clear();
     cout<<"Point cloud saved."<<endl;
     return 0;
 }
