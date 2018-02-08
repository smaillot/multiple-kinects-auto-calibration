#include <ros/ros.h>
#include <ctime>
// synchronization
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// message
#include <sensor_msgs/Image.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <multiple_kinects/line_detectionConfig.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace cv_bridge;
using namespace cv;

ros::Publisher pub1;
ros::Publisher pub2;

int cannyLowThreshold = 200;
int cannyHighThreshold = 400;
int cannyKernel = 3;
int houghResolution = 50;
int houghThreshold = 50;
int minLinLength = 50;
int maxLineGap = 10;

void dynrec_callback(multiple_kinects::line_detectionConfig &config, uint32_t level)
{
    cannyLowThreshold = config.cannyLowThreshold;
    cannyHighThreshold = config.cannyHighThreshold;
    houghResolution = config.houghResolution;
    houghThreshold = config.houghThreshold;
    minLinLength = config.minLinLength;
    maxLineGap = config.maxLineGap;
}

CvImagePtr image_processing(const CvImagePtr ptr1, const CvImagePtr ptr2)
{
    Mat img_1 = ptr1->image;
    Mat img_2 = ptr2->image;

    return ptr1;
}

CvImagePtr line_detection(const CvImagePtr ptr)
{
    Mat img = ptr->image;
    CvImagePtr output;
    output = ptr;

    Mat dst, cdst;
    Canny(img, dst, cannyLowThreshold, cannyHighThreshold, cannyKernel);
    cvtColor(dst, cdst, CV_GRAY2BGR);

    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI/houghResolution, houghThreshold, minLinLength, maxLineGap );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }

    //imshow("source", img);
    //imshow("detected lines", cdst);

    output->image = cdst;
    return output;
}

void rgb_callback(const ImageConstPtr& rgb1, const ImageConstPtr& rgb2)
{
    CvImagePtr cv_ptr1;
    CvImagePtr cv_ptr2;
    CvImagePtr output1;
    CvImagePtr output2;

    try
    {
      cv_ptr1 = toCvCopy(rgb1, image_encodings::BGR8);
      cv_ptr2 = toCvCopy(rgb2, image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    output1 = line_detection(cv_ptr1);
    output2 = line_detection(cv_ptr2);

    // Output modified video stream
    pub1.publish(output1->toImageMsg());
    pub2.publish(output2->toImageMsg());
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "cv_node");
    ros::NodeHandle nh;

    // dynamic reconfigure
    dynamic_reconfigure::Server<multiple_kinects::line_detectionConfig> server;
    dynamic_reconfigure::Server<multiple_kinects::line_detectionConfig>::CallbackType f;
    f = boost::bind(&dynrec_callback, _1, _2);
    server.setCallback(f);

    // Synchronize both kinects messages
    message_filters::Subscriber<Image> cam1(nh, "/cam1/qhd/image_color", 1);
    message_filters::Subscriber<Image> cam2(nh, "/cam2/qhd/image_color", 1);
    typedef sync_policies::ApproximateTime<Image, Image> KinectSync;
    Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
    sync.registerCallback(boost::bind(&rgb_callback, _1, _2));

    pub1 = nh.advertise<Image>("line_detection1", 1);
    pub2 = nh.advertise<Image>("line_detection2", 1);

    // Spin
    ros::spin();
}
