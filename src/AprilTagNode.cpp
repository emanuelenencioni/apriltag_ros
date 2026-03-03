// ros
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include "pose_estimation.hpp"
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// apriltag
#include <apriltag/apriltag.h>

#include <cmath>

#include "tag_functions.hpp"

#define IF(N, V) \
    if (assign_check(parameter, N, V)) continue;

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, CameraInfo>;
using namespace message_filters;
template <typename T>
void assign(const rclcpp::Parameter& parameter, T& var) {
    var = parameter.get_value<T>();
}

template <typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var) {
    var = parameter.get_value<T>();
}

template <typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var) {
    if (parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

rcl_interfaces::msg::ParameterDescriptor descr(const std::string& description, const bool& read_only = false) {
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

class AprilTagNode : public rclcpp::Node {
   public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

   private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    std::vector<std::pair<apriltag_family_t*, std::function<void(apriltag_family_t*)>>> tag_families_list;
    apriltag_detector_t* const td;

    // parameter
    std::mutex mutex;
    double tag_edge_size;  // used in the lock
    int max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;
    std::unordered_map<std::string, double> family_sizes;  // family name -> size
    std::string debug_image_name;

    // const image_transport::CameraSubscriber sub_cam;
    message_filters::Subscriber<Image> image_sub;
    message_filters::Subscriber<CameraInfo> info_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)

/*


sub_cam(image_transport::create_camera_subscription(
        this,
        this->get_node_topics_interface()->resolve_topic_name("image_rect"),
         std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
         declare_parameter("image_transport", "raw", descr({}, true)),
         rmw_qos_profile_sensor_data)),

*/

AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
    : Node("apriltag_ros_node", options),
      // parameter
      cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
      td(apriltag_detector_create()),
      // topics
      pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
      debug_img_pub(create_publisher<sensor_msgs::msg::Image>("debug_image", rclcpp::QoS(1))),
      tf_broadcaster(this) {
    rclcpp::QoS qos = rclcpp::QoS(5);
    image_sub.subscribe(this, "image", qos.get_rmw_qos_profile());
    info_sub.subscribe(this, "camera_info", qos.get_rmw_qos_profile());
    sync = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(5), image_sub, info_sub);
    sync->registerCallback(std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2));

    const auto tag_families_param =
        declare_parameter("families", std::vector<std::string>{"36h11"}, descr("list of tag families", true));
    const auto families_sizes_param =
        declare_parameter("families_sizes", std::vector<double>{},
                          descr("tag sizes per family (must match families array length)", true));
    tag_edge_size = declare_parameter("size", 0.05, descr("default tag size"));
    // get tag names, IDs and sizes
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames =
        declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // get method for estimating tag pose
    const std::string& pose_estimation_method =
        declare_parameter("pose_estimation_method", "pnp",
                          descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster), "
                                "set to \"\" (empty) to disable pose estimation",
                                true));

    if (!pose_estimation_method.empty()) {
        if (pose_estimation_methods.count(pose_estimation_method)) {
            estimate_pose = pose_estimation_methods.at(pose_estimation_method);
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown pose estimation method '" << pose_estimation_method << "'.");
        }
    }
    debug_image_name = std::string("debug_output");
    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));
    declare_parameter("detector.debug_image", std::string("debug_output"), descr("debug image name without extension"));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    if (!frames.empty()) {
        if (ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" +
                                     std::to_string(frames.size()) + ") mismatch!");
        }
        for (size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if (!sizes.empty()) {
        // use tag specific size
        if (ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" +
                                     std::to_string(sizes.size()) + ") mismatch!");
        }
        for (size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if (!families_sizes_param.empty()) {
        // use family-specific sizes
        if (tag_families_param.size() != families_sizes_param.size()) {
            throw std::runtime_error("Number of families (" + std::to_string(tag_families_param.size()) +
                                     ") and families_sizes (" + std::to_string(families_sizes_param.size()) +
                                     ") mismatch!");
        }
        for (size_t i = 0; i < tag_families_param.size(); i++) {
            family_sizes[tag_families_param[i]] = families_sizes_param[i];
        }
    }

    for (const auto& fam : tag_families_param) {
        if (tag_fun.count(fam)) {
            apriltag_family_t* fam_ptr = tag_fun.at(fam).first();
            apriltag_detector_add_family(td, fam_ptr);
            tag_families_list.push_back({fam_ptr, tag_fun.at(fam).second});
        } else {
            throw std::runtime_error("Unsupported tag family: " + fam);
        }
    }
}

AprilTagNode::~AprilTagNode() {
    apriltag_detector_destroy(td);
    for (auto& [fam_ptr, destructor] : tag_families_list) { destructor(fam_ptr); }
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) {
    // camera intrinsics for recttf2ified images
    const std::array<double, 4> intrinsics = {msg_ci->p[0], msg_ci->p[5], msg_ci->p[2], msg_ci->p[6]};

    // check for valid intrinsics
    const bool calibrated =
        msg_ci->width && msg_ci->height && intrinsics[0] && intrinsics[1] && intrinsics[2] && intrinsics[3];
    if (estimate_pose != nullptr && !calibrated) {
        RCLCPP_WARN_STREAM(get_logger(),
                           "The camera is not calibrated! Set 'pose_estimation_method' to \"\" (empty) to disable pose "
                           "estimation and this warning.");
    }

    // convert to 8bit monochrome image
    const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};
    double local_tag_edge_size = 0.0;
    // detect tags
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    bool debug = td->debug;
    local_tag_edge_size = tag_edge_size;
    mutex.unlock();

    if (profile) timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;
    RCLCPP_INFO(get_logger(), "Received image %s with %d detections", msg_img->header.frame_id.c_str(),
                zarray_size(detections));
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    RCLCPP_DEBUG(get_logger(), "detections: %d", zarray_size(detections));
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        RCLCPP_DEBUG(get_logger(), "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n", i,
                     det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);

        // ignore untracked tags
        if (!tag_frames.empty() && !tag_frames.count(det->id)) {
            RCLCPP_DEBUG(get_logger(), "detection %d ignored: not tracked", det->id);
            continue;
        }

        // reject detections with more corrected bits than allowed
        if (det->hamming > max_hamming) {
            RCLCPP_DEBUG(get_logger(), "detection %d rejected: hamming %d > max_hamming %d", det->id, det->hamming,
                         max_hamming);
            continue;
        }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
        msg_detections.detections.push_back(msg_detection);

        // 3D orientation and position
        if (estimate_pose != nullptr && calibrated) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_img->header;
            // set child frame name by generic tag name or configured tag name
            tf.child_frame_id = tag_frames.count(det->id)
                                    ? tag_frames.at(det->id)
                                    : std::string(det->family->name) + ":" + std::to_string(det->id);

            // Size resolution priority (lowest to highest):
            // 1. Global default size (local_tag_edge_size)
            // 2. Family-specific default size (family_sizes)
            // 3. Tag-ID-specific size (tag_sizes)
            double size = local_tag_edge_size;
            if (family_sizes.count(det->family->name)) { size = family_sizes.at(det->family->name); }
            if (tag_sizes.count(det->id)) { size = tag_sizes.at(det->id); }

            tf.transform = estimate_pose(det, intrinsics, size);
            tfs.push_back(tf);
        }
        if (debug) {
            cv::Mat debug_img_color = cv::imread(debug_image_name + ".pnm");
            if (!debug_img_color.empty()) {
                cv::Mat debug_img;
                cv::cvtColor(debug_img_color, debug_img, cv::COLOR_BGR2GRAY);
                // cv::Mat debug_img = cv::imread('output_img.pnm',-1);
                cv_bridge::CvImage img_bridge =
                    cv_bridge::CvImage(msg_img->header, sensor_msgs::image_encodings::MONO8, debug_img);
                debug_img_pub->publish(*img_bridge.toImageMsg());
            }
        }
    }
    pub_detections->publish(msg_detections);

    if (estimate_pose != nullptr) {
        tf_broadcaster.sendTransform(tfs);
        if (debug && tfs.size() > 0) {
            double dist = sqrt(pow(tfs[0].transform.translation.x, 2) + pow(tfs[0].transform.translation.y, 2) +
                               pow(tfs[0].transform.translation.z, 2));
            RCLCPP_INFO_STREAM(get_logger(), "tag distance: " << dist << ", z: " << tfs[0].transform.translation.z);
        }
    }
    apriltag_detections_destroy(detections);
}

rcl_interfaces::msg::SetParametersResult AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for (const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);
        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("detector.debug_image", debug_image_name)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
        IF("size", tag_edge_size)
    }

    mutex.unlock();

    result.successful = true;

    return result;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto aprilTag = std::make_shared<AprilTagNode>(options);
    rclcpp::Rate sleepRate(std::chrono::milliseconds(100));
    rclcpp::spin(aprilTag);
    rclcpp::shutdown();
    return 0;
}
