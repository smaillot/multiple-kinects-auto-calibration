#include <tf_node.h>

using namespace std;

string ref;
string frame;
float freq;
int transl_filter_type;
float transl_filter_weight;
int rot_filter_type;
float rot_filter_weight;
bool rejection;
float max_translation;
float max_rotation;

int prev_transl_method = 0;
int prev_rot_method = 0;
int it_transl = 1;
int it_rot = 1;
tf::Transform mem;
ros::Publisher pub;

tf::Vector3 origin(0, 0, 0);
tf::Quaternion identity_quat(0, 0, 0, 1);

void conf_callback(calib::TFConfig &config, uint32_t level)
{
    ROS_DEBUG("reconfigure");
    freq = config.freq;

    transl_filter_type = config.transl_filter_type;
    transl_filter_weight = config.transl_filter_weight;
    transl_filter_weight = pow(10, -transl_filter_weight);

    rot_filter_type = config.rot_filter_type;
    rot_filter_weight = config.rot_filter_weight;
    rot_filter_weight = pow(10, -rot_filter_weight);

    prev_transl_method = transl_filter_type;
    prev_transl_method = rot_filter_type;

    rejection = config.rejection;
    max_translation = config.max_translation / 1000;
    max_rotation = config.max_rotation * 3.14159 / 180;

    if (prev_transl_method != 2 && transl_filter_type == 2)
    {
        it_transl = 0;
        mem.setOrigin(origin);
    }
    if (prev_rot_method != 2 && rot_filter_type == 2)
    {
        it_rot = 0;
        mem.setRotation(identity_quat);
    }
}

tf::Transform filter_tf(tf::Transform mem, tf::StampedTransform new_tf, float ratio_transl, float ratio_rot)
{
    ROS_DEBUG("filter");
    tf::Transform result;
    tf::Vector3 res_o = (1 - ratio_transl) * mem.getOrigin() + ratio_transl * new_tf.getOrigin();
    tf::Quaternion res_q = mem.getRotation() * tfScalar(1 - ratio_rot) + new_tf.getRotation() * tfScalar(ratio_rot);
    result.setOrigin(res_o);
    result.setRotation(res_q);

    return result;
}

bool valid(tf::Transform mem, tf::Transform transform)
{
    if (mem.getOrigin().distance(transform.getOrigin()) > max_translation) return false;
    double rx1, ry1, rz1;
    double rx2, ry2, rz2;
    tf::Matrix3x3 mat1(mem.getRotation());
    mat1.getEulerYPR(rx1, ry1, rz1);
    tf::Matrix3x3 mat2(transform.getRotation());
    mat2.getEulerYPR(rx2, ry2, rz2);
    if (abs(rx1 - rx2) > max_rotation || abs(ry1 - ry2) > max_rotation || abs(rz1 - rz2) > max_rotation) return false;
    return true;
}

int main(int argc, char *argv[])
{
        ref = argv[1];
        frame = argv[2];
        string node_name = "tf_smoothing_" + frame;

    // Initialize ROS
        ros::init(argc, argv, node_name);
        ros::NodeHandle nh;
        ros::NodeHandle node_tf("/calib/tf/" + frame);

        pub = nh.advertise<calib::TF>("/calib/tf/" + frame + "_filtered", 1);

        dynamic_reconfigure::Server<calib::TFConfig> server(node_tf);
        dynamic_reconfigure::Server<calib::TFConfig>::CallbackType f;
        f = boost::bind(&conf_callback, _1, _2);
        server.setCallback(f);

    tf::TransformListener listener;

    // ROS loop
    static tf::TransformBroadcaster br;
    while (ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform(ref, frame, ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        float ratio_transl, ratio_rot;
        switch (transl_filter_type)
        {
            case 0:
                it_transl = 0;
                ratio_transl = 1;
                break;
            case 1:
                it_transl = 0;
                ratio_transl = transl_filter_weight;
                break;
            case 2:
                it_transl++;
                ratio_transl = 1 / (float)it_transl;
                break;
        }
        switch (rot_filter_type)
        {
            case 0:
                it_rot = 0;
                ratio_rot = 1;
                break;
            case 1:
                it_rot = 0;
                ratio_rot = rot_filter_weight;
                break;
            case 2:
                it_rot++;
                ratio_rot = 1 / (float)it_rot;
                break;
        }
        ROS_INFO_STREAM("Ratio at it (" << it_transl << " - " << it_rot << ") : " << ratio_transl << ", " << ratio_rot);
        
        tf::Transform new_tf = filter_tf(mem, transform, ratio_transl, ratio_rot);
        if (valid(mem, new_tf) || !rejection)
        {
            mem = new_tf; 
            br.sendTransform(tf::StampedTransform(mem, ros::Time::now(), ref, frame + "_filtered"));
        }
        else
        {
            ROS_ERROR("Outlying transform rejected !");
        }

        calib::TF msg;
        double rx, ry, rz;
        msg.tx = mem.getOrigin().getX();
        msg.ty = mem.getOrigin().getY();
        msg.tz = mem.getOrigin().getZ();
        msg.it_transl = it_transl;
        tf::Matrix3x3 mat(mem.getRotation());
        mat.getEulerYPR(rx, ry, rz);
        msg.rx = (float) 180 / 3.14159 * rx;
        msg.ry = (float) 180 / 3.14159 * ry;
        msg.rz = (float) 180 / 3.14159 * rz;
        msg.it_rot = it_rot;
        pub.publish(msg);

        ros::spinOnce();
		if (freq > 0)
		{
			ros::Rate loop_rate(freq);
			loop_rate.sleep();
		}
    }

    return 0;
}