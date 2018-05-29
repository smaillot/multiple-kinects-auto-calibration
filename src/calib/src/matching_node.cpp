#include "matching_node.h"
 
using namespace std;
using namespace message_filters;
 
float freq = 0;
ros::Publisher pub;
calib::Planes planes1;
calib::Planes planes2;
Matching* matchPtr;

void conf_callback(calib::NodeConfig &config, uint32_t level)
{
    freq = config.freq;
    switch(config.logger)
    {
      case 0:
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
        {
          ros::console::notifyLoggerLevelsChanged();
        }
        break;
      case 2:
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn)) 
        {
          ros::console::notifyLoggerLevelsChanged();
        }
        break;
      case 3:
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal)) 
        {
          ros::console::notifyLoggerLevelsChanged();
        }
        break;
      default:
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) 
        {
          ros::console::notifyLoggerLevelsChanged();
        }
    }
}

void planes_callback(const calib::PlanesConstPtr& planes1, const calib::PlanesConstPtr& planes2)
{
    matchPtr->update(planes1, planes2);
}

int main(int argc, char *argv[])
{
///////////
// init
    std::ofstream nullstream;
    std::clog.rdbuf(nullstream.rdbuf());

    string name1 = argv[1];
    string name2 = argv[2];
    string topic = "/calib/planes/";
    string topic1 = topic + name1 + "/planes";
    string topic2 = topic + name2 + "/planes"; 
    string node_name = (string) "transform_estimation_" + name1 + "_" + name2;
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("/calib/" + name1 + "_" + name2);
    ros::NodeHandle node_matching("/calib/" + name1 + "_" + name2 + "/matching");

    message_filters::Subscriber<calib::Planes> cam1_sub(nh, topic1, 1);
    message_filters::Subscriber<calib::Planes> cam2_sub(nh, topic2, 1);
    typedef sync_policies::ApproximateTime<calib::Planes, calib::Planes> KinectSync;
    Synchronizer<KinectSync> sync(KinectSync(10), cam1_sub, cam2_sub);
    sync.registerCallback(boost::bind(&planes_callback, _1, _2));

    dynamic_reconfigure::Server<calib::NodeConfig> server(nh);
    dynamic_reconfigure::Server<calib::NodeConfig>::CallbackType f;
    f = boost::bind(&conf_callback, _1, _2);
    server.setCallback(f);

  ///////////
// main code
  
  Matching match(&node_matching, name1, name2);
  matchPtr = &match; 
  dynamic_reconfigure::Server<calib::MatchingConfig> server_matching(node_matching);
  dynamic_reconfigure::Server<calib::MatchingConfig>::CallbackType f_match;
  f_match = boost::bind(&Matching::conf_callback, &match, _1, _2);
  server_matching.setCallback(f_match);
 
 ////////////
// ros loop
 
  while (ros::ok())
	{
		ros::spinOnce();
		if (freq > 0)
		{
			ros::Rate loop_rate(freq);
			loop_rate.sleep();
		}
	}
 
  return 0;
///////////
}