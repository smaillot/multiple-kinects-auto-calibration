#include <calib/icp_node.h> 
 
using namespace std; 
using namespace message_filters; 
 
string name; 
string target_topic;
string output_name;  
float frequency = 0; 
tf::TransformListener *listener; 
tf::TransformBroadcaster *br;
ros::Publisher pub_pc;
float subsize = 50;

bool enable;
bool autom;
bool reciproc;
float max_dist;
int max_it;
float eps;
pcl::VoxelGrid<Point> filter_voxel;

void conf_callback(calib::ICPConfig &config, uint32_t level)
{
    enable = config.enable;
    autom = config.autom;
    reciproc = config.reciproc;
    max_dist = config.max_dist / 1000;
    max_it = config.max_it;
    eps = pow(10, -config.epsilon);
    subsize = config.subsize / 1000;
}

pcPtr subsample(const pcPtr& input, float size)
{
    if (input->points.size() > 0)
    {
        filter_voxel.setInputCloud(input);
        filter_voxel.setLeafSize(size, size, size);
        pc_t* cloud = new pc_t; 
        filter_voxel.filter(*cloud);
        pcPtr output(cloud);
        if (output->points.size() > 0)
        {
            return output;
        }
        else
        {
            return input;
        }
    }
    else
    {
        return input;
    }
}

/** 
 * @brief Callback from kinects synchronization. 
 */ 
void pc_callback(const pcConstPtr& pc1, const pcConstPtr& pc2) 
{  
    pc_t new_pc;
    pcl_ros::transformPointCloud("world", *pc1, new_pc, *listener); 
    pcPtr subsampled(new pc_t(new_pc));
    if (subsize > 0)
    {
        subsampled = subsample(subsampled, subsize);
    }
    pc_t registered; 
    
    if (enable)
    {
        pcl::IterativeClosestPoint<Point, Point> icp; 
        icp.setInputSource(subsampled); 
        icp.setInputTarget(pc2); 
    
        if (!autom)
        {
            icp.setUseReciprocalCorrespondences(reciproc); 
            icp.setMaxCorrespondenceDistance(max_dist); 
            icp.setMaximumIterations(max_it); 
            icp.setTransformationEpsilon (eps); 
            // icp.setEuclideanFitnessEpsilon(1); 
        }

        icp.align(registered);
        ROS_DEBUG_STREAM("has converged:\t" << icp.hasConverged() << ", score:\t" << icp.getFitnessScore());
        
        tf::Transform transform;
        Eigen::Matrix4f mat = icp.getFinalTransformation();
        Eigen::Affine3d T;
        T.matrix() = mat.cast<double>();
        tf::transformEigenToTF(T, transform);
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name + "_icp"));

        pcl_ros::transformPointCloud(*pc1, registered, transform);
        listener->waitForTransform(registered.header.frame_id, "world", ros::Time(0), ros::Duration(5.0));
    }
    else    
    {
        registered = *pc1;
        tf::Transform transform;
        transform.setIdentity();
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name + "_icp"));
    }
 
    pub_pc.publish(registered); 
} 

int main(int argc, char *argv[]) 
{  
    // parsing arguments 
 
        string node_name; 
        name = argv[1]; 
        node_name = "icp_" + name; 
        target_topic = argv[2]; 
        
        // Initialize ROS 
        ros::init(argc, argv, node_name); 
        ros::NodeHandle nh; 
        ros::NodeHandle node_icp("/calib/" + name + "/icp");

        listener = new tf::TransformListener;
        br = new tf::TransformBroadcaster;
 
    // Synchronize both kinects messages 
        message_filters::Subscriber<pc_t> cam1(nh, "/calib/clouds/" + name, 1); 
        message_filters::Subscriber<pc_t> cam2(nh, target_topic, 1); 
        typedef sync_policies::ApproximateTime<pc_t, pc_t> KinectSync; 
        Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2); 
        sync.registerCallback(boost::bind(&pc_callback, _1, _2)); 

        pub_pc = nh.advertise<pc_t>("/calib/clouds/" + name + "/icp", 1);

        dynamic_reconfigure::Server<calib::ICPConfig> server(node_icp);
        dynamic_reconfigure::Server<calib::ICPConfig>::CallbackType f;
        f = boost::bind(&conf_callback, _1, _2);
        server.setCallback(f);
 
    // ROS loop 
    while (ros::ok()) 
    { 
         
        // sleep 
            if (frequency > 0) 
            { 
                ros::Rate loop_rate(frequency); 
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