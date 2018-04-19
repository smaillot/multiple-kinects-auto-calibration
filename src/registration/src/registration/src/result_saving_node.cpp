#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;
int i = 0;
float last1 = 0;
float change1;
float last2 = 0;
float change2;
ros::Publisher pub_tr;
ros::Publisher pub_rot;

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

string print_tf1(tf::StampedTransform transform)
{
    string output = "";
    tf::Vector3 t = transform.getOrigin();
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 a = q.getAxis();
    change1 = last1 - q.getAngle();
    if (change1 > 0.00001 || change1 < -0.00001)
    {
        output += patch::to_string(t.getX()) + "\t" + patch::to_string(t.getY()) + "\t" + patch::to_string(t.getZ()) + "\t";
        output += patch::to_string(a.getX()) + "\t" + patch::to_string(a.getY()) + "\t" + patch::to_string(a.getZ()) + "\t";
        output += patch::to_string(q.getAngle()) + "\n";
        last1 = q.getAngle();
        i++;
        ROS_DEBUG_STREAM("print #" + patch::to_string(i));
    }

    geometry_msgs::Vector3 tr_msg;
    geometry_msgs::Quaternion rot_msg;
    tr_msg.x = t.getX() * 1000;
    tr_msg.y = t.getY() * 1000;
    tr_msg.z = t.getZ() * 1000;
    rot_msg.x = a.getX() * 1000;
    rot_msg.y = a.getY() * 1000;
    rot_msg.z = a.getZ() * 1000;
    rot_msg.w = q.getAngle() * 180 / 3.14159;
    pub_tr.publish(tr_msg);
    pub_rot.publish(rot_msg);

    return output;
}

string print_tf2(tf::StampedTransform transform)
{
    string output = "";
    tf::Vector3 t = transform.getOrigin();
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 a = q.getAxis();
    change2 = last2 - q.getAngle();
    if (change2 > 0.00001 || change2 < -0.00001)
    {
        output += patch::to_string(t.getX()) + "\t" + patch::to_string(t.getY()) + "\t" + patch::to_string(t.getZ()) + "\t";
        output += patch::to_string(a.getX()) + "\t" + patch::to_string(a.getY()) + "\t" + patch::to_string(a.getZ()) + "\t";
        output += patch::to_string(q.getAngle()) + "\n";
        last2 = q.getAngle();
    }

    return output;
}

ofstream output_file1("/home/inhands-user3/catkin_ws/src/registration/src/registration/rec/registration/cam3_cam2.log");
//ofstream output_file2("/home/inhands-user3/catkin_ws/src/registration/src/registration/rec/registration/cam3_fix.log");

int main(int argc, char *argv[])
{
    // verbosity: debug
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
        {
            ros::console::notifyLoggerLevelsChanged();
        }

    // Initialize ROS
        ros::init(argc, argv, "histogram_printer_node");
        ros::NodeHandle nh;
        pub_tr = nh.advertise<geometry_msgs::Vector3>("registration/translation", 10);
        pub_rot = nh.advertise<geometry_msgs::Quaternion>("registration/rotation", 10);

    tf::TransformListener listener1;
    tf::TransformListener listener2;

    

    // ROS loop
    ros::Rate loop_rate(5.0);
    while (ros::ok())
    {
        tf::StampedTransform transform1;
        tf::StampedTransform transform2;

        try
        {
            listener1.waitForTransform("/registration_cam3_cam2", "/cam_center", ros::Time(0), ros::Duration(5.0));
            listener1.lookupTransform("/registration_cam3_cam2", "/cam_center", ros::Time(0), transform1);
            }
            catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        // try
        // {
        //     listener2.waitForTransform("/registration_cam3_fix", "/cam_center", ros::Time(0), ros::Duration(5.0));
        //     listener2.lookupTransform("registration_cam3_fix", "cam_center", ros::Time(0), transform2);
        //     }
        //     catch (tf::TransformException &ex) {
        //     ROS_ERROR("%s",ex.what());
        //     ros::Duration(1.0).sleep();
        //     continue;
        // }

        output_file1 << print_tf1(transform1);
        // output_file2 << print_tf2(transform2);

        // sleep
            ros::spinOnce();
            loop_rate.sleep();
    }

    return 0;
}