#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "slamBase.h"

using namespace std;

const double camera_factor = 1000;
const double camera_cx = 971.96923828125;
const double camera_cy = 593.4072875976562 ;
const double camera_fx = 1388.6114501953125;
const double camera_fy = 1388.6114501953125;

void callback(const sensor_msgs::Image::ConstPtr& rgb , const sensor_msgs::Image::ConstPtr& depth){
  static int frame = 0;


  cout<<"RGB"<<rgb->header<<endl;
  cout<<"Depth"<<depth->encoding<<endl;

  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(rgb, "bgr8");


  cv::Mat rgb_mat,dep_mat;
  rgb_mat = cv_ptr->image;

  cv_bridge::CvImageConstPtr cv_ptr2;


  cv_ptr2 = cv_bridge::toCvShare(depth,depth->encoding);

  cout<<"Done"<<depth->encoding<<endl;

  // (cv_ptr2->image).convertTo(dep_mat,CV_16UC1);

  // cv::normalize(dep_mat, dep_mat, 0, 1, cv::NORM_MINMAX,-1 );


  // for (int m = 0; m < dep_mat.rows; m++){
  //     for (int n=0; n < dep_mat.cols; n++)
  //     {
  //      if (dep_mat.ptr<short>(m)[n]!= 0)cout<<dep_mat.ptr<short>(m)[n]<<endl;
  //   }
  // }

  // cout<< depth->encoding<<endl;

// cout<<dep_mat.ptr<<endl;
// exit();

  PointCloud::Ptr cloud ( new PointCloud );

  // cout<<rgb_mat.size()<<endl;
  cout<<depth->encoding<<endl;

  dep_mat = cv_ptr->image;

  static ushort max_ =  0;
  static ushort min_ = 6553;

  for (int m = 0; m < dep_mat.rows; m++){
      for (int n=0; n < dep_mat.cols; n++)
      {

          // 获取深度图中(m,n)处的值

          ushort d = dep_mat.ptr<ushort>(m)[n];

          // cout<<d<<endl;

          // cout<<d<<endl;
          // d 可能没有值，若如此，跳过此点
          if (d <=0 || d!=d)
              continue;
          // d 存在值，则向点云增加一个点
          // cout<< " "<<m  <<" "<<n<<" "<<d<<endl;
          PointT p;

          max_ = max(d,max_);

          min_ = min(d,min_);
          // cout<<d<<endl;

          // 计算这个点的空间坐标
          p.z = double(d)/1000;
          p.x = (n - camera_cx) * p.z / camera_fx;
          p.y = (m - camera_cy) * p.z / camera_fy;




          // cout<<p.x<<" "<<p.y<<endl;
          // cout<<p.x<<" "<<p.y<< " "<<p.z<<endl;
          // p.z = 1;
          // p.x = 1;
          // p.y = 1;

          // 从rgb图像中获取它的颜色
          // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
          p.b = rgb_mat.ptr<uchar>(m)[n*3];
          p.g = rgb_mat.ptr<uchar>(m)[n*3+1];
          p.r = rgb_mat.ptr<uchar>(m)[n*3+2];

          // 把p加入到点云中
          cloud->points.push_back( p );
      }
    }
  cout<<"here"<<min_<<" "<<max_<<endl;
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cout<<"point cloud size = "<<cloud->points.size()<<endl;
  cloud->is_dense = false;
  pcl::io::savePCDFile( "/home/fan/test_dir/develop/src/RGB-D/pointcloud.pcd", *cloud );
  cloud->points.clear();
  cout<<"Point cloud saved."<<endl;

  // cout<<im<<endl;

  // cv::imwrite("haha.jpg",cv_bridge::toCvShare(msg, "bgr8")->image);
  // cv::waitKey(30);


  exit(0);

  // frame++;
  //
  // if(frame == 2) {
  //
  //
  //   exit(1);
  // }

}

int main( int argc, char** argv ){

  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;

  // image_transport::ImageTransport it(n);
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/zed/left/image_raw_color", 10);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/zed/depth/depth_registered", 10);
  message_filters::Subscriber<sensor_msgs::CameraInfo> came_sub(nh, "/zed/left/CameraInfo", 10);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(rgb_sub, depth_sub,10);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  // cv::startWindowThread();
  //
  // cv::namedWindow("view");

  // image_transport::Subscriber sub = it.subscribe("/zed/left/image_rect_color", 1000, chatterCallback);

  ros::spin();

  // cv::destroyWindow("view");

  return 0;
}
