/*************************************************************************
	> File Name: rtabmap_navigation.cpp
	> Author: 
	> Mail: 
	> Created Time: Mon 08 Dec 2014 05:24:42 PM CST
 ************************************************************************/
#include <ros/ros.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#define PI 3.141592653
int main(int argc , char** argv){

    ros::init(argc, argv, "rtabmap_navigation") ;
    ros::NodeHandle nh ;
	double goal_x_ = 0 , goal_y_ = 0 ;
	double r_scale = 1.0 , x_scale = 5.0 ;
	double roll = 0, pitch = 1, yaw = 3.14/180*90;
	double x=0 , y=0 , z=0 ; 
	double rotation=0 , distance=0 ,  angularOffset = 0 ;
    bool direction = 0 ;
	nh.getParam("goal_x", goal_x_);
	nh.getParam("goal_y",goal_y_);

    tf::TransformListener listener;
    ros::Publisher cmdPub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    ros::Rate rate(100.0);

    while (nh.ok()){
            tf::StampedTransform transform;
        try{
                  listener.lookupTransform("/map", "/base_link",ros::Time(0),transform);         
        }
        catch (tf::TransformException ex){
                  ROS_ERROR("%s",ex.what());
                  ros::Duration(1.0).sleep();
                
        }
		//ROS_INFO("transform.getOrigin().x()= %f,transform.getOrigin().y()= %f, transform.getOrigin().z()= %f",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
        //ROS_INFO("transform.getRotation().x()= %f,transform.getRotation().y()= %f , transform.getRotation().z()= %f, transform.getRotation.w()= %f",transform.getRotation.x(),transform.getRotation.y(),transform.getRotation.y(),transform.getRotation.w());
        
		x = transform.getRotation().x() ;
		y = transform.getRotation().y() ;
		z = transform.getRotation().z() ;
		transform.getBasis().getRPY(roll,pitch,yaw);
		std::cerr << "Position: " << "[" << x << "," << y << "," << z << "]" << std::endl;
        std::cerr << "Rotation: " << "["<< roll << "," << pitch << "," << yaw << "]" << std::endl;
        rotation = atan2(goal_y_ - y , goal_x_ - x);
        distance = sqrt(pow(goal_x_ -x , 2.0)+pow(goal_y_ - y,2.0));
        std::cerr <<"rotation= " << rotation << ",distance= "<< distance << std::endl ;
		
        if(abs(yaw -rotation) < PI){ 
            angularOffset = yaw - rotation ;
        }else if ((yaw -rotation)>PI){
            angularOffset = (yaw - rotation)-2*PI;
        }else{
            angularOffset = 2*PI + (yaw-rotation) ;
        }
		std::cerr << "direction =" << direction << std::endl ;
		std::cerr << "angularOffset =" << angularOffset << std::endl ;
        if(abs(angularOffset) > 0.1){
            direction = false ;
        }else{
            direction = true ;
        }
        if(!direction){
            cmd->angular.z = -0.05 * angularOffset ;
            std::cerr <<"angular.z= "  << cmd->angular.z << std::endl ;
        }else{
            cmd->angular.z = 0 ;
        }
        if((direction) && (distance > 0)){
			std::cerr << "Entering this control" << std::endl ;
            //ROS_INFO("Entering this control!");
            cmd->linear.x = 0.5*distance ;
        }else{
            cmd->linear.x = 0 ;
        }
		//transform.getBasis().getRPY(roll,pitch,yaw);
        //cmd->angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
        //cmd->linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2)); 	
        cmdPub_.publish(cmd);
        rate.sleep();
          
    }
    return 0 ;
}
