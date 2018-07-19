#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

float freq = 1;

YAML::Node load_config(string file)
{
    YAML::Node config;
    try 
    {
        config = YAML::LoadFile(file); // gets the root node
    }
    catch (YAML::BadFile bf) 
    {
        ROS_WARN("No configuration file found, a new one will be created");
        config = YAML::Load("");
    }
    return config;
}

int main(int argc, char *argv[])
{
        string source = argv[1];
        string target = argv[2];
        string ref = argv[3];
        string frame = source + "_" + target;
        string frame_yaml = frame;
        if (argc > 6)
        {
            frame_yaml = argv[4];
            ROS_INFO_STREAM("tf name in YAML file changes to " << frame_yaml);
        }
        string node_name = "publish_" + frame;

    // Initialize ROS
        ros::init(argc, argv, node_name);
        ros::NodeHandle nh;
    
    std::string path = ros::package::getPath("calib");
    string file = path + "/config/transform.yaml";
    YAML::Node config = load_config(file);

    // ROS loop
    static tf::TransformBroadcaster br;
    while (ros::ok())
    {
        tf::Transform transform;
        float tx = config[frame_yaml]["tx"].as<float>();
        float ty = config[frame_yaml]["ty"].as<float>();
        float tz = config[frame_yaml]["tz"].as<float>();
        float rx = config[frame_yaml]["rx"].as<float>();
        float ry = config[frame_yaml]["ry"].as<float>();
        float rz = config[frame_yaml]["rz"].as<float>();
        transform.setOrigin(tf::Vector3(tx, ty, tz));
        transform.setRotation(tf::createQuaternionFromRPY(rx, ry, rz));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ref, frame + "_filtered"));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ref, frame));

        ros::Rate loop_rate(freq);
        loop_rate.sleep();
    }

    return 0;
}