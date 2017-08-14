/*************************************************************************
    > File Name: src/jointPointCloud.cpp
    > Author: Xiang gao
    > Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Created Time: 2015年07月22日 星期三 20时46分08秒
 ************************************************************************/

#include<iostream>


#include "slamBase.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/core/eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main( int argc, char** argv )
{
    //read parameter from yaml file
    ParameterReader pd;
    FRAME frame1, frame2;

    //read depth and rgb information
    frame1.rgb = cv::imread( "/home/fan/test_dir/develop/src/rgb_png/5.png" );
    frame1.depth = cv::imread( "/home/fan/test_dir/develop/src/depth_png/5.png", -1);
    frame2.rgb = cv::imread( "/home/fan/test_dir/develop/src/rgb_png/200.png" );
    frame2.depth = cv::imread( "/home/fan/test_dir/develop/src/depth_png/200.png", -1 );

    cout<<"extracting features"<<endl;
    computeKeyPointsAndDesp( frame1);
    computeKeyPointsAndDesp( frame2);

    // read camera param from yaml file
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fy = 325.5;
    camera.fx = 253.5;
    camera.cx = 518.0;
    camera.cy = 519.0;
    camera.scale = 1000;

    vector< cv::DMatch > matches;
    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(5, 24, 2));

    matcher.match( frame1.desp, frame2.desp, matches );


    cout<<"Find total "<<matches.size()<<" matches."<<endl;

    vector< cv::DMatch > goodMatches;
    double minDis = 9999;

    //using distance threashold to filter some bad matches
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 10*minDis)
            goodMatches.push_back( matches[i] );
    }
    cout<<"good matches: "<<goodMatches.size()<<endl;
    vector<cv::Point3f> pts_obj;
    vector< cv::Point2f > pts_img;

    for (size_t i=0; i<goodMatches.size(); i++)
    {

        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;

        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        cv::Point3f pt( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        //  cv::Point3f pd;
        //  pd.z = double( pt.z ) / C.scale;
        //  pd.x = ( pt.x - C.cx) * pd.z / C.fx;
        //  pd.y = ( pt.y - C.cy) * pd.z / C.fy;

        pts_obj.push_back( pd );

    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;

    //performing pnp Ransanc
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );
    cout<<"inliers: "<<inliers.rows<<endl;
    cout<<"R="<<rvec<<endl;
    cout<<"t="<<tvec/(camera.scale) <<endl;

    RESULT_OF_PNP result;
    result.rvec = rvec;
    result.tvec = tvec/(camera.scale);
    result.inliers = inliers.rows;


    cv::Mat R;
    cv::Rodrigues( result.rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    // convert screw axis and rotation plus rotation into transformation
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    cout<<"rotational matrix "<<r<<endl;
    //normalize the rotational matrix
    Eigen::AngleAxisd angle(r);
    cout<<"translation"<<endl;
    Eigen::Translation<double,3> trans(result.tvec.at<double>(0,0), result.tvec.at<double>(0,1), result.tvec.at<double>(0,2));
    T = angle;
    T(0,3) = result.tvec.at<double>(0,0);
    T(1,3) = result.tvec.at<double>(0,1);
    T(2,3) = result.tvec.at<double>(0,2);

    cout<<"converting image to clouds"<<endl;
    PointCloud::Ptr cloud1 = image2PointCloud( frame1.rgb, frame1.depth, camera );
    PointCloud::Ptr cloud2 = image2PointCloud( frame2.rgb, frame2.depth, camera );

    // 合并点云
    cout<<"combining clouds"<<endl;
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *cloud1, *output, T.matrix() );
    *output += *cloud2;
    pcl::io::savePCDFile("/home/fan/test_dir/develop/src/RGB-D/src/data/result.pcd", *output);
    cout<<"Final result saved."<<endl;

    return 0;
}
