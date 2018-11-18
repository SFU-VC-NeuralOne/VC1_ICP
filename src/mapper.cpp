//
// Created by rakesh on 27/08/18.
//

#include <icp_slam/mapper.h>
#include <iostream>
using namespace std;

namespace icp_slam
{

 Mapper::Mapper(){
     //anything
 }
void Mapper::initMap(int width, int height, float resolution,
            double origin_x_meters, double origin_y_meters,
            uint8_t *pointer, unsigned char unknown_cost_value)
{
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_x_ = origin_x_meters;
    origin_y_ = origin_y_meters;
    map_.create(width_, height_, CV_8SC1);
    cout<<"map_ created"<<map_.rows<<endl;
    relative_map_.create(width_, height_, CV_8SC1);
    cout<<"relative_map_ created"<<map_.rows<<endl;
    for(int i=0; i<height_; i++)
    {
        for(int j=0; j<width_; j++)
        {
            map_.at<int8_t>(i,j)=NO_INFORMATION;
        }
    }
    
    for(int i=0; i<height_; i++)
    {
        for(int j=0; j<width_; j++)
            relative_map_.at<int8_t>(i,j)=0;
    }
    is_initialized_ = true;
    //cout<<"triggerred init!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<end;
}

int Mapper::updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                const tf::StampedTransform &pose)
{
    if(is_running == true)
    {
        return 0;
    }
    else
    {
        is_running = true;
    float initial_angle = tf::getYaw(pose.getRotation());

    int laser_size = (laser_scan->angle_max - laser_scan->angle_min)/laser_scan->angle_increment;
    float robot_x = pose.getOrigin().getX();
    float robot_y = pose.getOrigin().getY();
    int robot_grid_x;
    int robot_grid_y;
    convertToGridCoords(robot_x, robot_y, robot_grid_x, robot_grid_y);

    float scan_x;
    float scan_y;
    int scan_grid_x;
    int scan_grid_y;
    float r;
    float a;

  //cout<<"laser size"<<laser_size;
    cv::Mat scan_mat(laser_size, 2, CV_32F);
     for(int i=0 ;i<laser_size; i++)
    {
        r = laser_scan->ranges[i];
        a = laser_scan->angle_min + (laser_scan->angle_increment)*i + initial_angle;
        utils::polarToCartesian(r, a, scan_x, scan_y);
        scan_x = scan_x + robot_x;
        scan_y = scan_y + robot_y;
        //cout<<i<<"th item from original scan"<<"x "<< scan_x <<" y "<<scan_y<<endl;
        convertToGridCoords(scan_x, scan_y, scan_grid_x, scan_grid_y);
        //cout<<i<<"th item"<<"scan x "<< scan_grid_x <<" scan y "<<scan_grid_y<<" robot x "<<robot_grid_x<<" robot y "<<robot_grid_y<<endl;
        cv::LineIterator it(map_,
                            cv::Point(robot_grid_x, robot_grid_y),
                            cv::Point(scan_grid_x, scan_grid_y)
        );
        for(int j = 0; j < it.count; j++, ++it) {
            if(j== (it.count -1))
            {
                cv::Point point = it.pos(); 
                for (int a=point.x-1; a<=point.x+1; a++)
                {
                    for (int b=point.y-1; b<=point.y+1; b++)  
                    {
                        //cout<<"rela size"<<relative_map_.size()<<"a "<<a<<" b "<<b<<" bofre minus is"<<relative_map_.at<int8_t>(a, b)<<endl;
                        if(relative_map_.at<int8_t>(a,b)<126)
                        { 
                        relative_map_.at<int8_t>(a,b) = relative_map_.at<int8_t>(a,b)+1;
                        }
                        
                     }
                    
                }
            }
            else{
                cv::Point point = it.pos(); 
                if(relative_map_.at<int8_t>(point.x, point.y)>-126)
                    relative_map_.at<int8_t>(point.x, point.y) = relative_map_.at<int8_t>(point.x, point.y) - 1;
            }
        }
    }
    //cout<<"relative map size:"<<relative_map_.size<<endl;
    //cout<<"relative map for now \n"<<relative_map_<<endl;

    for (int i=0; i<height_; i++)
    {
        for(int j=0; j<width_; j++)
        {
            if(relative_map_.at<int8_t>(i,j) > free_threshold_)
                map_.at<int8_t>(i,j) =  LETHAL_OBSTACLE;
            else if(relative_map_.at<int8_t>(i,j) < -free_threshold_)
                map_.at<int8_t>(i,j) = FREE_SPACE;
            else
                map_.at<int8_t>(i,j) = NO_INFORMATION;
        }
    }
    is_running = false;
    }
    return 1;
}
cv::Mat Mapper::getMapCopy()
{
    cv::Mat new_mat(height_, width_, CV_8SC1);
    for(int i=0; i<height_; i++)
    {
        for(int j=0; j<width_; j++)
            new_mat.at<int8_t>(i,j)=map_.at<int8_t>(j,i);
    }

    //nav_msgs::OccupancyGrid new_map;


    return new_mat;

}

int Mapper::convertToGridCoords(double x, double y, int &grid_x, int &grid_y)
{
    grid_x = x/resolution_ + width_/2;
    grid_y = y/resolution_ + height_/2;
    if(grid_x<0)
        grid_x = 0;
    else if (grid_x>width_)
        grid_x = width_;
    
    if(grid_y<0)
        grid_y = 0;
    else if (grid_y>height_)
        grid_y = height_;

}

} // namespace icp_slam
