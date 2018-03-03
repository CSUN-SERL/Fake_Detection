#ifndef SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_
#define SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_

//#include <vector>
//#include <queue>
#include <string>

#include "ros/ros.h"
#include "new_detection_msgs/CompiledMessage.h"

//#include "std_msgs/Int8.h"
//#include "sensor_msgs/Image.h"
//#include "darknet_ros_msgs/BoundingBoxes.h"
//#include "detection_msgs/DetectionPointCloud.h"
//#include "detection_msgs/PointCloudImage.h"
//#include "detection_msgs/ProcessedVisualDetection.h"
//#include "detection_msgs/CompiledMessage.h"

//#include "visual_detection_tracker.h"

namespace sarwai {

  class ImageBoundingBoxMerger {
  public:

    ImageBoundingBoxMerger(std::string subscriptionTopic);
    ~ImageBoundingBoxMerger();
//    void TrackMagic();

  private:
    const unsigned BOXLENGTH = 70;
    //VisualDetectionTracker* tracking_handler_;
    ros::NodeHandle* m_nh;

    ros::Subscriber m_trackingSub;
    ros::Publisher m_visualDetectionPub;
    ros::Publisher m_boxStreamPubOne;
    ros::Publisher m_boxStreamPubTwo;
    ros::Publisher m_boxStreamPubThree;
    ros::Publisher m_boxStreamPubFour;

    void drawBoxesCallback(const new_detection_msgs::CompiledMessageConstPtr& msg);
    void drawBoxAndSendQuery(const new_detection_msgs::CompiledMessageConstPtr& msg, new_detection_msgs::Human human) const;
    void drawBoxAroundHuman(sensor_msgs::Image& image, new_detection_msgs::Human human, float fov) const;
    void sendBoxedStream(const new_detection_msgs::CompiledMessageConstPtr& msg) const;
    // ros::Subscriber bounding_box_sub_;
    // ros::Subscriber detection_flag_sub_;

    // ros::Subscriber raw_image_frame_sub_;

    //ros::Publisher visual_detection_pub_;
    //Queue hold series of 1s and 0s
    //std::queue<int> detection_flag_;  
    //Queue hold video frames of type sensor_msgs::Image
    //std::queue<sensor_msgs::Image> video_image_frames_; 
    //queue of bounding box information
//    std::queue<std::vector<darknet_ros_msgs::BoundingBox>> bounding_boxes_matrix_;  
    
    sensor_msgs::Image drawBox(sensor_msgs::Image image, new_detection_msgs::Human human);
//    void PublishMergedData(sensor_msgs::Image, darknet_ros_msgs::BoundingBox, unsigned robotId); 
//    void RunImageProcess(const detection_msgs::CompiledMessageConstPtr& msg);
//    void ImageAndBoundingBoxToPublishQueue(darknet_ros_msgs::BoundingBox,
//      sensor_msgs::Image);
//    void DrawRectAndPublishImage(const darknet_ros_msgs::BoundingBox &box, const sensor_msgs::Image &, unsigned robotId);
  };
}

#endif
