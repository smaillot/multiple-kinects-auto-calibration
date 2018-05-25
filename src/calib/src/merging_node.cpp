#include <merging_node.h>

using namespace std;
using namespace message_filters;

vector <string> inputs;
string name;
ros::Publisher pub_pc;
string output;

/**
 * @brief Callback from kinects synchronization.
 */
void pc_callback(const pcConstPtr& pc1, const pcConstPtr& pc2)
{
    pc_t input1 = *pc1;
    pc_t input2 = *pc2;
    pc_t merged = input1 + input2;
    pub_pc.publish(merged);
}

int main(int argc, char *argv[])
{

    // parsing arguments

		inputs.clear();

        inputs.push_back(argv[1]);
        inputs.push_back(argv[2]);
        output = argv[3];
        string node_name = "merging_" + output;

    // Initialize ROS
        ros::init(argc, argv, node_name);
        ros::NodeHandle nh;

        pub_pc = nh.advertise<pc_msg_t>("/calib/clouds/" + output, 1);

    // Synchronize both kinects messages
        message_filters::Subscriber<pc_t> cam1_sub(nh, inputs[0], 1);
        message_filters::Subscriber<pc_t> cam2_sub(nh, inputs[1], 1);
        typedef sync_policies::ApproximateTime<pc_t, pc_t> KinectSync;
        Synchronizer<KinectSync> sync(KinectSync(10), cam1_sub, cam2_sub);
        sync.registerCallback(boost::bind(&pc_callback, _1, _2));


    // ROS loop
    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}