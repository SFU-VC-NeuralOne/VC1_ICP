//
// Created by rakesh on 17/08/18.
//

#include <icp_slam/utils.h>
#include<iostream>
using namespace std;

namespace icp_slam
{
namespace utils
{

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan)
{
  // TODO
  int laser_size = (scan->angle_max - scan->angle_min)/scan->angle_increment;
  //cout<<"laser size"<<laser_size;
  cv::Mat point_mat(laser_size, 2, CV_32F);
  //float[laser_size][2] points;
  
  for(int i=0 ;i<laser_size; i++)
  {
    float x;
    float y;
    float r = scan->ranges[i];
    float a = scan->angle_min + (scan->angle_increment)*i;
    //polarToCartesian(r, a, &x, &y);
    polarToCartesian(r, a, x, y);
    //cout<<i<<"th item"<<"x "<< x <<" y "<<y<<endl;
    //print this
    point_mat.at<float>(i,0) = x;
    point_mat.at<float>(i,1) = y;
  }

  return point_mat; 

}

cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  cv::Mat transformed_point_mat =  T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

cv::Mat re_order (cv::Mat &original_mat, std::vector<int> new_order )
{

  for(int i = 0; i < new_order.size(); i++)
  {

  }
}

} // namespace utils
} // namespace icp_slam