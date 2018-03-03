#include <iostream>

#include "image_bounding_box_merger.h"
//#include "sensor_msgs/Image.h"
//#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace sarwai {

  ImageBoundingBoxMerger::ImageBoundingBoxMerger(std::string subscriptionTopic) {
    m_nh = new ros::NodeHandle();

    m_trackingSub = m_nh->subscribe(subscriptionTopic, 1000, &ImageBoundingBoxMerger::drawBoxesCallback, this);
    m_visualDetectionPub = m_nh->advertise<new_detection_msgs::CompiledMessage>("/sarwai_detection/detection_processeddetection", 1000);
    m_boxStreamPubOne = m_nh->advertise<sensor_msgs::Image>("/robot1/camera/rgb/image_boxed", 1000);
    m_boxStreamPubTwo = m_nh->advertise<sensor_msgs::Image>("/robot2/camera/rgb/image_boxed", 1000);
    m_boxStreamPubThree = m_nh->advertise<sensor_msgs::Image>("robot3/camera/rgb/image_boxed", 1000);
    m_boxStreamPubFour = m_nh->advertise<sensor_msgs::Image>("robot4/camera/rgb/image_boxed", 1000);
//    this->image_frame_sub_ = this->nh_->subscribe(
//      "/compiled_ros_message", 1000, &ImageBoundingBoxMerger::RunImageProcess, this);


    //Publishes to visual_detection topic
//      this->visual_detection_pub_ = this->nh_->advertise<detection_msgs::ProcessedVisualDetection>(
//        "/sarwai_detection/detection_processeddetection", 1000);
  }
  

  ImageBoundingBoxMerger::~ImageBoundingBoxMerger() {
    //empty
  }
  
  void ImageBoundingBoxMerger::drawBoxesCallback(const new_detection_msgs::CompiledMessageConstPtr& msg) {
    // Draw boxes for each new query
    for(unsigned i = 0; i < msg->humanQueries.size(); ++i) {
      for(unsigned h = 0; h < msg->humans.size(); ++h) {
        if(msg->humans[h].id == msg->humanQueries[i]) {
          drawBoxAndSendQuery(msg, msg->humans[h]);
          break;
        }
      }
    }
    // Draw boxes around each person in one frame
	sendBoxedStream(msg);
  }

  void ImageBoundingBoxMerger::drawBoxAndSendQuery(const new_detection_msgs::CompiledMessageConstPtr& msg, new_detection_msgs::Human human) const {
    sensor_msgs::Image imageCopy(msg->img);

    drawBoxAroundHuman(imageCopy, human, msg->fov);

    new_detection_msgs::CompiledMessage queryMsg;
    queryMsg.header = msg->header;
    queryMsg.img = imageCopy;
    queryMsg.robot = msg->robot;

    this->m_visualDetectionPub.publish(queryMsg);
  }

  void ImageBoundingBoxMerger::drawBoxAroundHuman(sensor_msgs::Image& image, new_detection_msgs::Human human, float fov) const {
    unsigned yCoord = image.height - (BOXLENGTH / 2);
    unsigned xCoord = ((-1 * (human.angleToRobot / (fov / image.width))) + (image.width / 2)) - (BOXLENGTH / 2);
    cv_bridge::CvImagePtr cvImage;
    cvImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    cv::Mat imageMatrix = cvImage->image;
    cv::Point topLeftCorner = cv::Point(xCoord, yCoord);
    cv::Point bottomRightCorner = cv::Point(xCoord + BOXLENGTH, yCoord + BOXLENGTH);
    cv::rectangle(imageMatrix, topLeftCorner, bottomRightCorner, 3);
    image = *(cv_bridge::CvImage(image.header, "bgr8", imageMatrix).toImageMsg());
  }

  void ImageBoundingBoxMerger::sendBoxedStream(const new_detection_msgs::CompiledMessageConstPtr& msg) const {
    sensor_msgs::Image imageCopy(msg->img);
    for(unsigned i = 0; i < msg->humans.size(); ++i) {
      drawBoxAroundHuman(imageCopy, msg->humans[i], msg->fov);
    }
    
    unsigned id = msg->robot;
    if(id == 1) {
      m_boxStreamPubOne.publish(imageCopy);
    }
    else if(id == 2) {
      m_boxStreamPubTwo.publish(imageCopy);
    }
    else if(id == 3) {
      m_boxStreamPubThree.publish(imageCopy);
    }
    else {
      m_boxStreamPubFour.publish(imageCopy);
    }
  }

//  void ImageBoundingBoxMerger::PublishMergedData(
//    sensor_msgs::Image image, darknet_ros_msgs::BoundingBox box, unsigned int robotId) {
//      detection_msgs::ProcessedVisualDetection outgoing_detection_msg;
//      //Set image info to custom message detection_msgs::ProcessedVisualDetection
//      outgoing_detection_msg.image = image; 
//      //Set bounding box info to custom message detection_msgs::ProcessedVisualDetection
//      outgoing_detection_msg.bounding_box = box;
//      outgoing_detection_msg.robotId = robotId;
//      this->visual_detection_pub_.publish(outgoing_detection_msg);  
//  }

//  void ImageBoundingBoxMerger::RunImageProcess(const detection_msgs::CompiledMessageConstPtr& msg) {
//    std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes = msg->boxes.boundingBoxes;
//    sensor_msgs::Image master_image = msg->image;
//    unsigned robotId = msg->robotId;
//    for (int i = 0; i < bounding_boxes.size(); i++) {
//      DrawRectAndPublishImage(bounding_boxes[i], master_image, robotId);    
//    }
//  }

  // Function draws box around the detected image
//  void ImageBoundingBoxMerger::DrawRectAndPublishImage( 
//    const darknet_ros_msgs::BoundingBox &box, const sensor_msgs::Image &image, unsigned robotId) {
//      // Create a value copy of the image, to be modified later
//      sensor_msgs::Image image_copy = image; // @TODO: is this an unnecessary copy?
//      // Create an OpenCV image matrix from the ROS Image msg
//      cv_bridge::CvImagePtr cv_image;
//      cv_image = cv_bridge::toCvCopy(image_copy, sensor_msgs::image_encodings::BGR8);
//      cv::Mat image_matrix = cv_image->image;
//      cv::Point top_left_corner = cv::Point(box.xmin, box.ymin);  
//      cv::Point bottom_right_corner = cv::Point(box.xmax, box.ymax);
//      // Draw the bounding box on the image
//      cv::rectangle(image_matrix, top_left_corner, bottom_right_corner, 2);
//     sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_matrix).toImageMsg();
//     image_copy = *image_msg;
//      // Reassigns header value as the transition to OpenCV and back drops the header data.
//      // In particular, we are interested in the timestamp of the image.
//      image_copy.header = image.header;
//      PublishMergedData(image_copy, box, robotId);
//  }
}
