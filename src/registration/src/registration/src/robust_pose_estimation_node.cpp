#include <registration/robust_pose_estimation_node.h>

using namespace std;
using namespace message_filters;
/*
*	args parser
*
*	node frequency
*	input topics namespace
*	output topics namespace
*/

const string inputs[2] = {"/cam1", "/cam2"};
const int n_inputs = sizeof(inputs) / sizeof(*inputs);
const string sub_topic_name = "/reconstruction/point_clouds";
const string pub_topic_name = "/reconstruction/point_clouds/merged";
float frequency = 2;
ros::Publisher pub_pc_reg;
ros::Publisher pub_pc_inliers;

bool enable;
int max_iter;
int n_samples;
int randomness;
double sim_th;
double corr_th;
double inliers_frac;

// Types
    typedef pcl::PointNormal PointNT;
    typedef pcl::PointCloud<PointNT> PointCloudT;
    typedef pcl::FPFHSignature33 FeatureT;
    typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
    typedef pcl::PointCloud<FeatureT> FeatureCloudT;
    typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

/**
 * @brief Returns the publishing topic of a given camera.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_topic_name(int input_number)
{
	return sub_topic_name + inputs[input_number];
}

/**
 * @brief Callback from kinects synchronization.
 */
void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    if (enable)
    {
        // Point clouds
            pcl::PCLPointCloud2 cam1;
            pcl::PCLPointCloud2 cam2;
            pcl_conversions::toPCL(*pc1, cam1);
            pcl_conversions::toPCL(*pc2, cam2);
            PointCloudT::Ptr temp1(new PointCloudT);
            PointCloudT::Ptr temp2(new PointCloudT);
            pcl::fromPCLPointCloud2(cam1, *temp1);
            pcl::fromPCLPointCloud2(cam2, *temp2);
            PointCloudT::Ptr temp2_aligned (new PointCloudT);
            FeatureCloudT::Ptr temp2_features (new FeatureCloudT);
            FeatureCloudT::Ptr temp1_features (new FeatureCloudT);

        // Estimate normals for scene
            ROS_DEBUG("Estimating scene normals...");
            pcl::NormalEstimationOMP<PointNT,PointNT> nest;
            nest.setRadiusSearch(0.04);
            nest.setInputCloud(temp1);
            nest.compute(*temp1);

        // Estimate features
            ROS_DEBUG("Estimating features...");
            FeatureEstimationT fest;
            fest.setRadiusSearch(0.1);
            fest.setInputCloud(temp2);
            fest.setInputNormals(temp2);
            fest.compute(*temp2_features);
            fest.setInputCloud(temp1);
            fest.setInputNormals(temp1);
            fest.compute(*temp1_features);

        // Perform alignment
            ROS_DEBUG("Starting alignment...");
            pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
            align.setInputSource(temp2);
            align.setSourceFeatures(temp2_features);
            align.setInputTarget(temp1);
            align.setTargetFeatures(temp1_features);
            align.setMaximumIterations(max_iter); // Number of RANSAC iterations
            align.setNumberOfSamples(n_samples); // Number of points to sample for generating/prerejecting a pose
            align.setCorrespondenceRandomness(randomness); // Number of nearest features to use
            align.setSimilarityThreshold(sim_th); // Polygonal edge length similarity threshold
            align.setMaxCorrespondenceDistance(corr_th); // Inlier threshold
            align.setInlierFraction(inliers_frac); // Required inlier fraction for accepting a pose hypothesis
            {
                pcl::ScopeTime t("Alignment");
                align.align(*temp2_aligned);
            }

        if (align.hasConverged())
        {
            // extract inliers
                PointCloudT::Ptr inliers;
                pcl::PointIndices::Ptr point_inliers(new pcl::PointIndices);
                point_inliers->indices = align.getInliers();

                pcl::ExtractIndices<PointNT> extract_pos;
                extract_pos.setInputCloud(temp2_aligned);
                extract_pos.setIndices(point_inliers);
                extract_pos.setNegative(false);
                extract_pos.filter(*inliers);
            // publish
                sensor_msgs::PointCloud2 pc_reg;
                sensor_msgs::PointCloud2 inliers_msg;

                pcl::toROSMsg(*temp2_aligned, pc_reg);
                pcl::toROSMsg(*inliers, inliers_msg);

                pub_pc_reg.publish(pc_reg);
                pub_pc_inliers.publish(inliers_msg);
                ROS_DEBUG_STREAM(align.getInliers().size());
                ROS_DEBUG_STREAM(temp2->size());
        }
        else
        {
            ROS_DEBUG("Failed to align point clouds.");
        }
    }
}

void dynrec_callback(registration::PoseEstimationConfig &config, uint32_t level) 
{
    enable = config.enable;
    max_iter = config.max_iter;
    n_samples = config.n_samples;
    randomness = config.randomness;
    sim_th = config.sim_th;
    corr_th = config.corr_th / 1000;
    inliers_frac = config.inliers_frac / 100;
}

int main(int argc, char *argv[])
{
	// verbosity: debug
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

	// Initialize ROS
		ros::init(argc, argv, "robust_pose_estimation_node");
        ros::NodeHandle nh;

    // Dynamic reconfigure
        dynamic_reconfigure::Server<registration::PoseEstimationConfig> server;
        dynamic_reconfigure::Server<registration::PoseEstimationConfig>::CallbackType f;
        f = boost::bind(&dynrec_callback, _1, _2);
        server.setCallback(f);

    // Synchronize both kinects messages
        message_filters::Subscriber<sensor_msgs::PointCloud2> cam1(nh, get_topic_name(0), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> cam2(nh, get_topic_name(1), 1);
        typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> KinectSync;
        Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
        sync.registerCallback(boost::bind(&pc_callback, _1, _2));

        pub_pc_reg = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name + "/cam2_reg", 1);
        pub_pc_inliers = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name + "/cam2_inliers", 1);

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