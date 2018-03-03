#ifndef SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_
#define SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_


#include <string>

#include "ros/ros.h"
#include "new_detection_msgs/CompiledMessage.h"


namespace sarwai {

  class ImageBoundingBoxMerger {
  public:

    ImageBoundingBoxMerger(std::string subscriptionTopic);
    ~ImageBoundingBoxMerger();

  private:
    const unsigned BOXLENGTH = 70;

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

    sensor_msgs::Image drawBox(sensor_msgs::Image image, new_detection_msgs::Human human);

  };
}

#endif
