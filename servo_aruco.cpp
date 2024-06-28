#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h> 
#include<moveit/planning_scene_monitor/planning_scene_monitor.h>
#include<moveit/planning_scene/planning_scene.h>
#include<std_msgs/Float64MultiArray.h>
#include "get_pos/d2.h"
#include <cmath>
#include <vector>
#include <tuple>
#include <Eigen/Eigen>
using namespace Eigen;

using namespace std;
geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped target_pose_1;
geometry_msgs::PoseStamped current_pose0;
geometry_msgs::PoseStamped init_aruco;
// double euler0[3];
double rx0,ry0,rz0;
Isometry3d T_arm=Isometry3d::Identity();
Matrix3d R_arm=Matrix3d::Identity();
Vector3d t_arm;


double real_z=0.426;
double fx=601.3097534179688;
double fy=601.372802734375;
double kp=1;
double roll=0;
double pitch=0;
double yaw=0;
double deltayaw=0;
double yaw0=0;


void handposeCallback(const geometry_msgs::PoseStamped::ConstPtr& pos)
 {
    std::cout<<"enter calback"<<endl;
    // // double deltax,deltay;
    // // deltax=(state_msg->pos[0]-320.0)*real_z/fx;
    // // deltay=(state_msg->pos[1]-240.0)*real_z/fy;
    // target_pose_1.pose.position.x=pos->pose.position.x;
    // target_pose_1.pose.position.y=pos->pose.position.y;
    // target_pose_1.pose.position.z=pos->pose.position.z-0.40;

    // std::cout<<"state_msg.pose.x"<<target_pose_1.pose.position.x<<std::endl;
    // std::cout<<"state_msg.pose.y"<<target_pose_1.pose.position.y<<std::endl;
    // std::cout<<"state_msg.pose.z"<<target_pose_1.pose.position.z<<std::endl;
    
    // // target_pose_1.pose.orientation.x=pos->pose.orientation.x;
    // // target_pose_1.pose.orientation.y=pos->pose.orientation.y;
    // // target_pose_1.pose.orientation.z=pos->pose.orientation.z;
    // // target_pose_1.pose.orientation.w=pos->pose.orientation.w;

    // //init_aruco.pose=pos->pose;
    // tuple<double,double,double> euler=quatertoeuler(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
    // tie (roll,pitch,yaw)=euler;
    // deltayaw=yaw-yaw0;
    // yaw0=yaw;

    Quaterniond tar_q(pos->pose.orientation.w,pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z),q0(0,1,0,0);
    tar_q.normalize();
    q0.normalize();
    cout<<"1.tar_q"<<tar_q.coeffs().transpose()<<endl;
    cout<<"2.q0"<<q0.coeffs().transpose()<<endl;
    Vector3d tar_t(pos->pose.position.x,pos->pose.position.y,pos->pose.position.z),t0(0,0,0.4);
    // Isometry3d Tctar(tar_q);
    Isometry3d Tc0(q0);
    // Tctar.pretranslate(tar_t);
    Tc0.pretranslate(t0);
    cout<<"3.Tc0"<<Tc0.matrix()<<endl;
    Isometry3d T0c=Tc0.inverse();
    Vector3d pose_tar_in_0=T0c*tar_t;
    cout<<"3.1.pose_tar_in_0"<<pose_tar_in_0.transpose()<<endl;
    
    Matrix3d r_tar = tar_q.toRotationMatrix();
    cout<<"3.2.r_tar"<<r_tar.transpose()<<endl;
    
    Matrix3d r0=q0.toRotationMatrix();
    //modify
    Matrix3d r_tar_in_0=r0.inverse()*r_tar;
    
    cout<<"3.3.r0"<<r0.transpose()<<endl;
    cout<<"3.4.r_tar_in_0"<<r_tar_in_0<<endl;

    Matrix4d T;
    T.setIdentity();
    T.block<3,3>(0,0) = r_tar_in_0;
    T.topRightCorner<3, 1>() = pose_tar_in_0;
    
    Isometry3d T0tar=Isometry3d(T);
    // T0tar.rotate(r_tar_in_0);
    // T0tar.pretranslate(pose_tar_in_0);
    cout<<"3.5.T0tar"<<endl<<T0tar.matrix()<<endl;
    
    //modify
   // T_arm=T0c*T0tar*Tc0;
    Isometry3d T_camera;
    T_camera=Tc0*T0tar*T0c;

    Isometry3d T1=Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    
    T1(0,0) = -0.6, T1(0,1) = -0.8, T1(0,2) = 0, T1(0,3) = 0;
    T1(1,0) = 0.8, T1(1,1) = 0.6, T1(1,2) = 0, T1(1,3) = 0;
    T1(2,0) = 0, T1(2,1) = 0, T1(2,2) = 1, T1(2,3) = 0;
    T1(3,0) = 0, T1(3,1) = 0, T1(3,2) = 0, T1(3,3) = 1;

    T_arm=T1*T_camera*T1.inverse();
    R_arm=T_arm.rotation();
    t_arm=T_arm.translation();
    cout<<"4.T_arm"<<T_arm.matrix()<<endl;
    cout<<"5.R_arm"<<R_arm.transpose()<<endl;
    cout<<"6.t_arm"<<t_arm.transpose()<<endl;


 }


int main(int argc, char** argv)
{
    ros::init(argc, argv ,"servo_aruco");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //ros::Publisher joint_values_publisher=n.advertise<std_msgs::Float64MultiArray>("/joint_values",1);
    moveit::planning_interface::MoveGroupInterface move_group("jaka_zu5");
    
    
    ros::Subscriber hand_pose_subscriber =n.subscribe("aruco_single/pose",1,handposeCallback);

    // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // planning_scene_monitor->startStateMonitor();
    // planning_scene_monitor->startSceneMonitor();
    // planning_scene_monitor->startWorldGeometryMonitor();
    
    
    std::string end_link=move_group.getEndEffectorLink();
    std::string reference_frame="Link_0";
    move_group.setPoseReferenceFrame(reference_frame);
    //current_pose0 =move_group.getCurrentPose(end_link);
    
    
    // tuple<double,double,double> euler00=quatertoeuler(current_pose0.pose.orientation.x,current_pose0.pose.orientation.y,current_pose0.pose.orientation.z,current_pose0.pose.orientation.w);
    // tie<double,double,double> (rx0,ry0,rz0)=euler00;
    

    move_group.setPlannerId("RRT");


    while(ros::ok())
    {


    geometry_msgs::PoseStamped current_pose =move_group.getCurrentPose(end_link);
    //ROS_INFO_STREAM("Current pose0: " <<endl<< current_pose0);
    ROS_INFO_STREAM("Current pose: " << endl<< current_pose);
    
    // double angle=getrotdir(current_pose);
    // ROS_INFO_STREAM("ANGLE"<<endl<<angle);
    // double euler[3]={rx0,ry0,rz0-angle};
    // // std::vector<double> quer[4];
    // // quer=eulerToQuaternion(euler[2],euler[1],euler[0]);
    // //double quer[4];
    // double x,y,z,w;
    // tuple<double,double,double,double> qua=eulerToQuaternion(euler[2],euler[1],euler[0]);
    // tie (x,y,z,w)=qua;
    // //quer={x,y,z,w};
    Quaterniond cur_q(current_pose.pose.orientation.w,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z); 
    cur_q.normalize();
    Vector3d cur_t(current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
    Isometry3d cur_T=Isometry3d::Identity();
    
    Matrix3d cur_r=cur_q.toRotationMatrix();
    cur_T.rotate(cur_r);
    cur_T.pretranslate(cur_t);
    Isometry3d T_TAR=Isometry3d::Identity();
    //mofify
    T_TAR=cur_T*T_arm;
    Vector3d t_TAR=T_TAR.translation();
    cout<<"t_TAR"<<t_TAR.transpose()<<endl;
    Matrix3d R_TAR=T_TAR.rotation();
    Quaterniond q(R_TAR);
    cout<<"while.3.q"<<q.coeffs().transpose()<<endl;

    // target_pose.pose.position.x=current_pose.pose.position.x+t_arm(0);
    // target_pose.pose.position.y=current_pose.pose.position.y+t_arm(1);
    // target_pose.pose.position.z=current_pose.pose.position.z+t_arm(2);
    // target_pose.pose.orientation.x=current_pose.pose.orientation.x+target_pose_1.pose.orientation.x;
    // target_pose.pose.orientation.y=current_pose.pose.orientation.y+target_pose_1.pose.orientation.y;
    // target_pose.pose.orientation.z=current_pose.pose.orientation.z+target_pose_1.pose.orientation.z;
    // target_pose.pose.orientation.w=current_pose.pose.orientation.w;
    // target_pose.pose.orientation.x=quer[0];
    // target_pose.pose.orientation.y=quer[1];
    // target_pose.pose.orientation.z=quer[2];
    // target_pose.pose.orientation.w=quer[3];
    target_pose.pose.position.x=t_TAR(0);
    target_pose.pose.position.y=t_TAR(1);
    target_pose.pose.position.z=t_TAR(2);
    target_pose.pose.orientation.x=q.x();
    target_pose.pose.orientation.y=q.y();
    target_pose.pose.orientation.z=q.z();
    target_pose.pose.orientation.w=q.w();
    

    // target_pose.pose.position.z=0.44;
    // target_pose.pose.orientation.x=0.707;
    // target_pose.pose.orientation.y=0.707;
    // target_pose.pose.orientation.z=0;
    // target_pose.pose.orientation.w=0;

    ROS_INFO_STREAM("TARGET:"<<endl<<target_pose);
    move_group.setPoseTarget(target_pose.pose);
     
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      
    moveit::planning_interface::MoveItErrorCode success=move_group.plan(my_plan);
    ROS_INFO("Plan (pose goal)%s",success?"":"FAIED");
    if(success)
    {
        move_group.execute(my_plan);
        ROS_INFO("SUCCESS");
    }
    //sleep(10);
    }
    ros::shutdown();
    return 0;
}




 






