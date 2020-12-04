#include <ros/ros.h>
#include<iostream>
#include<geometry_msgs/Pose.h>
#include<sensing_node/cameraStatus.h>
//#include<sensor_dummy/objectpos.h>
#include<sensing_node/LocateObjects.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

std::vector<Eigen::Matrix4d> V_T_cam_obj;

void DetectPoses()
{
    ros::NodeHandle n;

    ros::service::waitForService("sensing_node/cameraStatus");
    ros::ServiceClient camera_status_clientOn = n.serviceClient<sensing_node::cameraStatus>("sensing_node/cameraStatus");
    sensing_node::cameraStatus camera_status_srvOn;
    camera_status_srvOn.request.enable = true;
    camera_status_clientOn.call(camera_status_srvOn);

    ros::Duration(2).sleep();


    ros::service::waitForService("sensing_node/LocateObjects");
    ros::ServiceClient locate_objects_client = n.serviceClient<sensing_node::LocateObjects>("sensing_node/LocateObjects");
    sensing_node::LocateObjects locate_objects_srv;
    locate_objects_client.call(locate_objects_srv);
    ROS_INFO("Get world state service performed sucessfully !");
    std::vector<geometry_msgs::Pose> pose;
    pose.resize(locate_objects_srv.response.ObjPoses.size());
    pose=locate_objects_srv.response.ObjPoses;
//converts the object poses into rotation matrices (Transformation) w.r.t the camera frame.
    std::vector<Eigen::Vector3d> Vposition;
    std::vector<Eigen::Quaterniond> Vquaternion;
    std::vector<Eigen::Matrix3d> Vrotmatrix;
    Vposition.resize(pose.size());
    Vrotmatrix.resize(pose.size());
    Vquaternion.resize(pose.size());
    V_T_cam_obj.resize(pose.size());
    for(unsigned int i=0;i<pose.size();i++)
    {
        Vposition[i]<<pose[i].position.x,pose[i].position.y,pose[i].position.z;
        Vquaternion[i].x()= pose[i].orientation.x;
        Vquaternion[i].y()= pose[i].orientation.y;
        Vquaternion[i].z()= pose[i].orientation.z;
        Vquaternion[i].w()= pose[i].orientation.w;
        Vrotmatrix[i]=Vquaternion[i].toRotationMatrix();
        V_T_cam_obj[i]<<          Vrotmatrix[i](0,0), Vrotmatrix[i](0,1), Vrotmatrix[i](0,2),Vposition[i](0),
                                  Vrotmatrix[i](1,0), Vrotmatrix[i](1,1), Vrotmatrix[i](1,2),Vposition[i](1),
                                  Vrotmatrix[i](2,0), Vrotmatrix[i](2,1), Vrotmatrix[i](2,2),Vposition[i](2),
                                        0,                   0 ,                  0 ,              1;
    }



//    for(unsigned  int i=0;i<pose.size();i++)
//    {
//    std::cout<<pose[i]<<std::endl;
//    std::cout<<V_T_cam_obj[i]<<std::endl;

//    }
    //ros::Duration(2).sleep();

    ros::service::waitForService("sensing_node/cameraStatus");
    ros::ServiceClient camera_status_clientOff = n.serviceClient<sensing_node::cameraStatus>("sensing_node/cameraStatus");
    sensing_node::cameraStatus camera_status_srvOff;
    camera_status_srvOff.request.enable = false;
    camera_status_clientOff.call(camera_status_srvOff);
}

//This function will compute the object pose in the world frame.
void ComputeObjectPoseinWorldFrame()
{

    Eigen::VectorXd q(7);
    q(0)=0;               //j1
    q(1)=-0.8762;             //j2
    q(2)=0.0;               //j7
    q(3)=-0.37878+ M_PI/2.0;   //j3
    q(4)=0.798;              //j4    //+ M_PI/2.0;   // Add the offset from the Kautham kin model to the inner IK library kin model
    q(5)=1.93705;              //j5
    q(6)=0.0;             //j6
    Eigen::Matrix4d A1,A2,A3,A4,A5,A6,A7;

    A1 << cos(q(0)), -sin(q(0)), 0, 0,       sin(q(0)), cos(q(0)), 0, 0,        0, 0, 1, 0,                        0, 0, 0, 1;
    A2 << cos(q(1)), -sin(q(1)), 0, 30,      0, 0, -1, 0,                       sin(q(1)), cos(q(1)), 0, 100,      0, 0, 0, 1;
    A3 << cos(q(2)), -sin(q(2)), 0, -30,     0, 0, 1, 172.83,                   -sin(q(2)), -cos(q(2)), 0, 0,      0, 0, 0, 1;
    A4 << -sin(q(3)), -cos(q(3)), 0, -41.88, 0, 0, -1, 0,                       cos(q(3)), -sin(q(3)), 0, 78.73,   0, 0, 0, 1;
    A5 << 0, 0, 1, 164.61,                   -cos(q(4)), sin(q(4)), 0, -40.5,   -sin(q(4)), -cos(q(4)), 0, 0,      0, 0, 0, 1;
    A6 << cos(q(5)), -sin(q(5)), 0, -27,     0, 0, -1, 0,                       sin(q(5)), cos(q(5)), 0, 100.39,   0, 0, 0, 1;
    A7 << cos(q(6)), -sin(q(6)), 0, 27,      0, 0, 1, 29,                       -sin(q(6)), -cos(q(6)), 0, 0,      0, 0, 0, 1;


    Eigen::Matrix4d T1 = A1;
    Eigen::Matrix4d T2 = T1*A2;
    Eigen::Matrix4d T3 = T2*A3;
    Eigen::Matrix4d T4 = T3*A4;
    Eigen::Matrix4d T5 = T4*A5;
    Eigen::Matrix4d T6 = T5*A6;
    Eigen::Matrix4d T7 = T6*A7;

    // Set in metres
    for (unsigned int i=0; i<3; ++i){
        T1(i,3) /= 1000.0;
        T2(i,3) /= 1000.0;
        T3(i,3) /= 1000.0;
        T4(i,3) /= 1000.0;
        T5(i,3) /= 1000.0;
        T6(i,3) /= 1000.0;
        T7(i,3) /= 1000.0;
    }

    //TRANSFORMATION BETWEEN ROBOT WORLD AND ELBOW
    Eigen::Matrix4d T;
    T << -0.57156, -0.104847, 0.813834,   0.05355,
         0.616978, -0.708787, 0.341993,    0.0725,
         0.540978,  0.697587, 0.469803,   0.41492,
              0.0,       0.0,      0.0,       1.0;

   //TCP transformation wrt the base
    Eigen::Matrix4d T_tcp = T*T7;

    Eigen::Matrix4d T_tcp_camera;

    T_tcp_camera<< 1,  0,  0,  0,
                   0,  1,  0,  -0.054,
                   0,  0,  1,  0.089,
                   0,  0,  0,  1;

    for(unsigned int i=0;i<V_T_cam_obj.size();i++)
    {
        std::cout<<"Object Transformation is"<<std::endl;
        Eigen::Matrix4d T_world_obj = T_tcp*T_tcp_camera*V_T_cam_obj[i];
        std::cout<<T_world_obj<<std::endl;
    }
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "sensing_node");
    ros::NodeHandle n;

    DetectPoses();
    ComputeObjectPoseinWorldFrame();


    ros::spin();

    return 0;
}

//ros::service::waitForService("sensing_node/cameraStatus");
//ros::ServiceClient camera_status_clientOn = n.serviceClient<ar_track_alvar::cameraStatus>("ar_track_alvar/cameraStatus");
//ar_track_alvar::cameraStatus camera_status_srvOn;
//camera_status_srvOn.request.enable = true;
//camera_status_clientOn.call(camera_status_srvOn);

//ros::Duration(2).sleep();


//ros::service::waitForService("sensing_node/LocateObjects");
//ros::ServiceClient locate_objects_client = n.serviceClient<ar_track_alvar::LocateObjects>("ar_track_alvar/LocateObjects");
//ar_track_alvar::LocateObjects locate_objects_srv;
//locate_objects_client.call(locate_objects_srv);
//ROS_INFO("Get world state service performed sucessfully !");
//std::vector<geometry_msgs::Pose> pose;
//pose.resize(locate_objects_srv.response.ObjPoses.size());
//pose=locate_objects_srv.response.ObjPoses;
//for(unsigned  int i=0;i<pose.size();i++)
//std::cout<<pose[i]<<std::endl;

////ros::Duration(2).sleep();

//ros::service::waitForService("ar_track_alvar/cameraStatus");
//ros::ServiceClient camera_status_clientOff = n.serviceClient<ar_track_alvar::cameraStatus>("ar_track_alvar/cameraStatus");
//ar_track_alvar::cameraStatus camera_status_srvOff;
//camera_status_srvOff.request.enable = false;
//camera_status_clientOff.call(camera_status_srvOff);
