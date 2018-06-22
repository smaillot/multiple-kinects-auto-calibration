#include <calib/stitching_node.h> 
 
using namespace std; 
using namespace message_filters; 
using namespace cv;
using namespace cv::detail;
 
string name_cam1;
string name_cam2;
string name;
tf::TransformListener *listener;
tf::TransformBroadcaster *br;
float frequency = 0;


bool try_use_gpu = true;
// string features_type = "surf";
// float match_conf = 0.3;
// float surf_hess_th;
// int n_oct;
// int n_oct_desc;
// int n_layers;
// int n_layers_desc;
// int n_orb_features;
// string estimator_type = "affine";


// string ba_cost_func = "reproj";
// string ba_refine_mask = "xxxxx";
// bool do_wave_correct = true;
// WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;
// string warp_type = "spherical";
// string seam_find_type = "gc_color";
// float conf_thresh = 1.f;
// double seam_work_aspect = 1;
// int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
// double compose_megapix = -1;
// double work_megapix = 0.6;
// double seam_megapix = 0.1;
// int blend_type = Blender::MULTI_BAND;
// float blend_strength = 5;


Stitcher::Mode mode = Stitcher::PANORAMA;
vector<Mat> imgs(2);
image_transport::Publisher pub1;
image_transport::Publisher pub2;
image_transport::Publisher pubout;


void conf_callback(calib::StitchConfig &config, uint32_t level)
{
    try_use_gpu = config.gpu;
    // if (config.features_type == 0) features_type = "surf";
    // else features_type = "orb";
    // match_conf = config.match_conf;
    // surf_hess_th = config.surf_hess_th;
    // n_oct = config.n_oct;
    // n_oct_desc = config.n_oct_desc;
    // n_layers = config.n_layers;
    // n_layers_desc = config.n_layers_desc;
    // n_orb_features = config.n_orb_features;
    // if (config.estimator == 0) estimator_type == "affine";
    // else estimator_type == "homography";
}


/** 
 * @brief Callback from kinects synchronization. 
 */ 
void callback(const ImConstPtr& im1, const ImConstPtr& im2) 
{  
    // Receive
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;
    try
    {
      cv_ptr1 = cv_bridge::toCvCopy(im1, sensor_msgs::image_encodings::BGR8);
      cv_ptr2 = cv_bridge::toCvCopy(im2, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    imgs[0] = Mat(cv_ptr1->image).clone();
    imgs[1] = Mat(cv_ptr2->image).clone();


    double work_scale = 1, seam_scale = 1, compose_scale = 1;
    bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;

    // Process
    Mat pano;
    Ptr<FeaturesFinder> finder;

//    vector<ImageFeatures> features(2);
//     if (features_type == "surf")
//     {
// #ifdef HAVE_OPENCV_XFEATURES2D
//         if (try_use_gpu && cuda::getCudaEnabledDeviceCount() > 0)
//             finder = makePtr<SurfFeaturesFinderGpu>(surf_hess_th, n_oct, n_layers, n_oct_desc, n_layers_desc);
//         else
// #endif
//                 finder = makePtr<SurfFeaturesFinder>(surf_hess_th, n_oct, n_layers, n_oct_desc, n_layers_desc);
//     }
//     else if (features_type == "orb")
//     {
//         finder = makePtr<OrbFeaturesFinder>(Size(3,1), n_orb_features);
//     }
//     else
//     {
//         cout << "Unknown 2D features type: '" << features_type << "'.\n";
//         return;
//     }

//     int i = 0;
//     (*finder)(imgs[i], features[i]);
//     features[i].img_idx = i;
//     ROS_INFO_STREAM("Features in image #" << i+1 << ": " << features[i].keypoints.size());
//     cv_ptr1->image = imgs[i].clone();

//     i = 1;
//     (*finder)(imgs[i], features[i]);
//     features[i].img_idx = i;
//     ROS_INFO_STREAM("Features in image #" << i+1 << ": " << features[i].keypoints.size());
//     cv_ptr2->image = imgs[i].clone();

//     Mat full_img;
//     vector<Size> full_img_sizes(2);
//     double seam_work_aspect = 1;

//     for (int i = 0; i < 2; ++i)
//     {
//         full_img = imgs[i];
//         full_img_sizes[i] = full_img.size();
//     }

//     finder->collectGarbage();


//     vector<MatchesInfo> pairwise_matches;
//     vector<DMatch> matches;
//     Ptr<FeaturesMatcher> matcher;
//     matcher = makePtr<AffineBestOf2NearestMatcher>(false, try_use_gpu, match_conf);
//     (*matcher)(features, pairwise_matches);

//     for (int j = 0; j < pairwise_matches.size(); j++)
//     {
//         for (int k = 0; k < pairwise_matches[i].matches.size(); k++)
//         {
//             matches.push_back(pairwise_matches[i].matches[k]);
//         }
//     }
//     drawMatches(imgs[0], features[0].keypoints, imgs[1], features[1].keypoints, matches, pano);

//     matcher->collectGarbage();

    Ptr<Stitcher> stitcher = Stitcher::create(mode, try_use_gpu);
    stitcher->setRegistrationResol(-1); /// 0.6
    stitcher->setSeamEstimationResol(-1);   /// 0.1
    stitcher->setCompositingResol(-1);   //1
    stitcher->setPanoConfidenceThresh(-1);   //1
    stitcher->setWaveCorrection(true);
    stitcher->setWaveCorrectKind(detail::WAVE_CORRECT_HORIZ);
    Stitcher::Status status = stitcher->stitch(imgs, pano);
    if (status != Stitcher::OK)
    {
        ROS_ERROR_STREAM("Can't stitch images, error code = " << status);
        return;
    }

    // Publish
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pano).toImageMsg();
    pubout.publish(msg);
} 

int main(int argc, char *argv[]) 
{  
    // parsing arguments 
 
        string node_name; 
        name_cam1 = argv[1]; 
        name_cam2 = argv[2]; 
        name = name_cam1 + "_" + name_cam2;
        node_name = "stitching_" + name; 
        string target_topic = argv[3];
        
        // Initialize ROS 
        ros::init(argc, argv, node_name); 
        ros::NodeHandle nh; 
        ros::NodeHandle node_stitch("/calib/" + name + "/stitching");

        listener = new tf::TransformListener;
        br = new tf::TransformBroadcaster;
 
    // Synchronize both kinects messages 
        message_filters::Subscriber<Image_t> cam1(nh, "/" + name_cam1 + "/hd/image_color", 1); 
        message_filters::Subscriber<Image_t> cam2(nh, "/" + name_cam2 + "/hd/image_color", 1); 
        typedef sync_policies::ApproximateTime<Image_t, Image_t> KinectSync; 
        Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2); 
        sync.registerCallback(boost::bind(&callback, _1, _2)); 

        image_transport::ImageTransport it(nh);
        pub1 = it.advertise("/calib/images/" + name_cam1, 1);
        pub2 = it.advertise("/calib/images/" + name_cam2, 1);
        pubout = it.advertise("/calib/images/" + name, 1);

        dynamic_reconfigure::Server<calib::StitchConfig> server(node_stitch);
        dynamic_reconfigure::Server<calib::StitchConfig>::CallbackType f;
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