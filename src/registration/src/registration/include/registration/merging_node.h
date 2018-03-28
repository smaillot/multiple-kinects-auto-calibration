// std
    #include <string>
// tf
	#include <tf/transform_listener.h>
// PCL specific includes
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl_ros/transforms.h>
	#include <pcl/common/transforms.h>
	#include <pcl/filters/radius_outlier_removal.h>
	#include <pcl/filters/conditional_removal.h>
	#include <pcl/sample_consensus/method_types.h>
	#include <pcl/sample_consensus/model_types.h>
	#include <pcl/segmentation/sac_segmentation.h>
	#include <pcl/filters/extract_indices.h>
// tf
	#include <tf/LinearMath/Transform.h>
// synchronization
	#include <message_filters/subscriber.h>
	#include <message_filters/time_synchronizer.h>
	#include <message_filters/sync_policies/approximate_time.h>
// dyn rec
	#include <dynamic_reconfigure/server.h>
	#include <registration/MergingConfig.h>

	namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}