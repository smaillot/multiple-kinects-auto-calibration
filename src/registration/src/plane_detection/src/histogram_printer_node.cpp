#include <plane_detection/histogram_printer_node.h>

using namespace std;
using namespace sensor_msgs;
/*
*   args parser
*
*   node frequency
*   input topics namespace
*   output topics namespace
*/

const string inputs[] = {"/fix", "/cam3"};
const int n_inputs = sizeof(inputs) / sizeof(*inputs);
const string sub_topic_name = "/reconstruction/planes";
float frequency = 0;
int planes = 3;
vector <ros::Subscriber> sub;
int resolution = 32;


/**
 * @brief Returns the publishing topic of a given camera.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_topic_name(int input_number, int plane)
{
    return sub_topic_name + inputs[input_number] + "/plane" + patch::to_string(plane);
}

/**
 * @brief Callback from kinects synchronization.
 */
void pc_callback_rgb_hist(const PointCloud2ConstPtr& pc, int cam, int plane)
{
    pcl::PCLPointCloud2* cloudPtr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*pc, *cloudPtr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloudPtr, *cloud);
    vector< vector<int> > histogram(resolution, vector<int>(3));
    int n = pc->data.size();
    const Eigen::Vector3f maxrgb(255.0,255.0,255.0);
    Eigen::Vector3f divisor_rgb = maxrgb/((float)resolution-1);
    for (int i = 0; i < cloud->size(); i++)
    {
        Eigen::Vector3i rgb = cloud->at(i).getRGBVector3i();
        Eigen::Vector3f bin_rgb = rgb.cast<float>().array()/divisor_rgb.array();
        for (int j=0; j < 3; j++)
        {
            bin_rgb(j, 0) = floor(bin_rgb(j));
        }
        for (int color=0; color<3; color++)
        {
            int k = (int)bin_rgb(color, 0);
            histogram[k][color]++;
        }
    }
    
    int ms = (int)time(0)*1000;
    string filename = "/home/inhands-user3/catkin_ws/src/registration/src/plane_detection/hist" + inputs[cam] + "/plane" + patch::to_string(plane) + "/plane_hist_" + patch::to_string(resolution) + "_" + patch::to_string(ms) + ".txt";
    ofstream output_file(filename.data());
    ostream_iterator<int> output_iterator(output_file, "\t");
    output_file << resolution << "\n";
    for ( int i = 0 ; i < histogram.size() ; i++ ) 
    {
        copy(histogram.at(i).begin(), histogram.at(i).end(), output_iterator);
        output_file << '\n';
    }
    ROS_DEBUG("histogram printed !");
}

/**
 * @brief Callback from kinects synchronization.
 */
void pc_callback_3d_hist(const PointCloud2ConstPtr& pc, int cam, int plane)
{
    pcl::PCLPointCloud2* cloudPtr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*pc, *cloudPtr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloudPtr, *cloud);
    Array3D histogram(resolution, resolution, resolution, 0);
    int n = pc->data.size();
    const Eigen::Vector3f maxrgb(255.0,255.0,255.0);
    Eigen::Vector3f divisor_rgb = maxrgb/((float)resolution-1);
    for (int i = 0; i < cloud->size(); i++)
    {
        Eigen::Vector3i rgb = cloud->at(i).getRGBVector3i();
        Eigen::Vector3f bin_rgb = rgb.cast<float>().array()/divisor_rgb.array();
        for (int j=0; j < 3; j++)
        {
            bin_rgb(j, 0) = floor(bin_rgb(j));
        }
        int k = (int)bin_rgb(0, 0);
        int l = (int)bin_rgb(1, 0);
        int m = (int)bin_rgb(2, 0);
        histogram(k, l, m)++;
    }
    
    int ms = (int)time(0)*1000;
    string filename = "/home/inhands-user3/catkin_ws/src/registration/src/plane_detection/hist" + inputs[cam] + "/plane" + patch::to_string(plane) + "/plane_hist_" + patch::to_string(resolution) + "_" + patch::to_string(ms) + ".txt";
    ofstream output_file(filename.data());
    ostream_iterator<int> output_iterator(output_file, "\t");
    output_file << resolution << "\n";
    for ( int i = 0 ; i < resolution ; i++ ) 
    {
        for(int j = 0; j < resolution; j++)
        {
            for (int k = 0; k < resolution; k++)
            {
                if (histogram(i, j, k) != 0)
                {
                    output_file << patch::to_string(i) << " " << patch::to_string(j) << " " << patch::to_string(k) << " " << patch::to_string(histogram(i, j, k)) << '\n';
                }
            }
        }
    }
    ROS_DEBUG_STREAM("histogram printed ! " + inputs[cam] + " plane" + patch::to_string(plane));
}

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

    for (int i = 0; i < n_inputs; i++)
    {
        for (int j = 0; j < planes; j++)
        {
            ROS_DEBUG_STREAM("Subscribing to " << get_topic_name(i, j+1));
            sub.push_back(ros::Subscriber(nh.subscribe<PointCloud2>(get_topic_name(i, j+1), 1, boost::bind(pc_callback_3d_hist, _1, i, j+1))));
        }
    }

    // ROS loop
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        
        // sleep
            if (frequency > 0)
            {
                loop_rate.sleep();
                ros::spinOnce();
            }
            else
            {
                ros::spin();
            }
    }

    return 0;
}