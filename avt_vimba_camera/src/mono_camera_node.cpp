/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <avt_vimba_camera/mono_camera_node.hpp>
#include <avt_vimba_camera_msgs/srv/load_settings.hpp>
#include <avt_vimba_camera_msgs/srv/save_settings.hpp>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
using namespace std;
using namespace std::placeholders;

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>


vector<uint64_t> e_preprocess;
vector<uint64_t> start_preprocess;	
vector<uint64_t> end_preprocess;
vector<uint64_t> e_inference;
vector<uint64_t> start_inference;
vector<uint64_t> end_inference;
vector<uint64_t> e_postprocess;
vector<uint64_t> start_postprocess;
vector<uint64_t> end_postprocess;
vector<uint64_t> start_ethernet;
vector<uint64_t> start_preethernet;
vector<uint64_t> end_preethernet;
vector<uint64_t> e_preethernet;
vector<uint64_t> start_while;
vector<uint64_t> end_while;
vector<uint64_t> e_while;
vector<uint64_t> end_while_max;
vector<uint64_t> e_while_max;
int img_num=0;

std::ostringstream file_name;

namespace avt_vimba_camera
{
MonoCameraNode::MonoCameraNode() : Node("camera"), api_(this->get_logger()), cam_(std::shared_ptr<rclcpp::Node>(dynamic_cast<rclcpp::Node * >(this)))
{
  this->LoadParams();

  // GigE Camera : Frame callback
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCameraNode::FrameCallback, this, _1));

  // Image Selection
  this->image_selection_ = std::make_shared<ImageSelection>(this->node_index_, this->number_of_nodes_, this->local_inference_fps_, this->timestamp_margin_milisecond_);

  // Object Detection
  this->inference_ = std::make_shared<Darknet>(0.2, const_cast<char*>(dnn_cfg_path_.c_str()), const_cast<char*>(dnn_weight_path_.c_str()));
  
  // ROS2 : QoS
  const rclcpp::QoS system_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // ROS2 : Image Selection Synchronize
  this->cluster_synchronize_publisher_ = this->create_publisher<std_msgs::msg::Header>("/cluster/synchronize", system_qos);
  this->cluster_synchronize_subscriber_ = this->create_subscription<std_msgs::msg::Header>("/cluster/synchronize", system_qos, std::bind(&MonoCameraNode::ImageSelectionSynchronize, this, _1));

  // ROS2 : Publish detections to monitor
  this->detections_publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/detections", system_qos);

  // Image Selection : Synchronize
  this->synchronization_cnt_ = 0;
  this->cluster_flag_ = false;
}

MonoCameraNode::~MonoCameraNode()
{
  cam_.stop();
}

void MonoCameraNode::ImageSelectionSynchronize(std_msgs::msg::Header::SharedPtr base_timestamp)
{
  try {
    if (std::stoi(base_timestamp->frame_id) == 1)  // get only node 1's timestamp 
    {
      this->image_selection_->RegisterBaseTimestamp(rclcpp::Time(base_timestamp->stamp).seconds());
      this->cluster_flag_ = true;
    }
  }
  catch (int error)
  {
    std::cerr << "image synchronize index failure";
    return;
  }
}

void MonoCameraNode::LoadParams()
{
  ip_ = this->declare_parameter("ip", "");
  guid_ = this->declare_parameter("guid", "");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");
  frame_id_ = this->declare_parameter("frame_id", "");
  ptp_offset_ = this->declare_parameter("ptp_offset", 0);

  // Cluster Information
  node_index_ = this->declare_parameter("node_index", 0);

  // Image Selection
  number_of_nodes_ = this->declare_parameter("number_of_nodes", 3);
  local_inference_fps_ = this->declare_parameter("local_inference_fps", 5.0);
  timestamp_margin_milisecond_ = this->declare_parameter("timestamp_margin_milisecond", 0.05);

  // Image Selection : Synchronize
  convert_frame_ = this->declare_parameter("convert_frame", 5);

  // Object Detection
  dnn_cfg_path_ = this->declare_parameter("dnn_cfg_path", "/home/avees/object_detection/src/cluster-object-detection/avt_vimba_camera/include/objectdetection/darknet/cfg/yolov4.cfg");
  dnn_weight_path_ = this->declare_parameter("dnn_weight_path", "/home/avees/weights/yolov4.weights");

  RCLCPP_INFO(this->get_logger(), "[Initialize] Parameters loaded");
}

uint64_t MonoCameraNode::get_time_in_ms() 
{
  rclcpp::Time now = this->get_clock()->now();
  uint64_t nanosecond = now.nanoseconds(); 
  return nanosecond/1000;
}

void MonoCameraNode::Start()
{
  // Start Vimba & list all available cameras
  api_.start();

  // Start camera
  cam_.start(ip_, guid_, frame_id_, camera_info_url_);
  cam_.startImaging();
}

void MonoCameraNode::FrameCallback(const FramePtr& vimba_frame_ptr)
{
  RCLCPP_INFO(this->get_logger(), "=== AVEES - Cluster-based Object Detection System with Scalable Performance for Autonomous Driving ===");

  rclcpp::Time node_start_time = this->get_clock()->now();
  RCLCPP_INFO(get_logger(),"FrameCallback called at %.3f sec", this->get_clock()->now().seconds());

  sensor_msgs::msg::Image img;
  if (api_.frameToImage(vimba_frame_ptr, img))
  {
    // Set time stamp
    VmbUint64_t frame_timestamp;
    vimba_frame_ptr->GetTimestamp(frame_timestamp);
    img.header.stamp = rclcpp::Time(cam_.getTimestampRealTime(frame_timestamp) * 1.0e+9) + rclcpp::Duration(ptp_offset_, 0);
    RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image timestamp : %lf sec.", this->node_index_, rclcpp::Time(img.header.stamp).seconds());

    if (this->cluster_flag_)
    {
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Cluster mode", this->node_index_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Local mode", this->node_index_);
    }

    // Image Selection
    RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Expect timestamp : %lf sec.", this->node_index_, this->image_selection_->GetEstimatedTimestamp());
    if (this->image_selection_->IsSelfOrder(rclcpp::Time(img.header.stamp).seconds()) == true)
    {
      // Synchronization
      if (this->node_index_ == 1)
      {
        if (this->synchronization_cnt_ == this->convert_frame_)
        {
          std_msgs::msg::Header base_timestamp;
          base_timestamp.stamp = img.header.stamp;
          base_timestamp.frame_id = std::to_string(this->node_index_);
          this->cluster_synchronize_publisher_->publish(base_timestamp);
        }
        else if (this->synchronization_cnt_ < this->convert_frame_)
        {
          this->synchronization_cnt_ += 1;
        }
      }

      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Take this image", this->node_index_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Drop this image", this->node_index_);
      return;  // terminate this iteration
    }

    // sensor_msgs::msg::image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
    cv::Mat color_image;
    cv::cvtColor(cv_ptr->image, color_image, cv::COLOR_BayerRG2RGB);
    
    // Object Detection - Preprocess
    start_preprocess.push_back(get_time_in_ms());
    this->inference_->Preprocess(color_image);
    end_preprocess.push_back(get_time_in_ms());
    
    // Object Detection - DNN Inference
    start_inference.push_back(get_time_in_ms());
    this->inference_->Inference();
    end_inference.push_back(get_time_in_ms());
    
    // Object Detection - Postprocess
    start_postprocess.push_back(get_time_in_ms());
    std::vector<ObjectDetection> detections = this->inference_->Postprocess();
    end_postprocess.push_back(get_time_in_ms());
    
    // Ethernet Publisher
    vision_msgs::msg::Detection2DArray detections_ros2_msg;
    detections_ros2_msg.header = img.header;
    detections_ros2_msg.header.frame_id = std::to_string(this->node_index_);
    
    for (size_t i = 0; i < detections.size(); i++)
    {
      vision_msgs::msg::Detection2D detection_ros2_msg;
      detection_ros2_msg.tracking_id = std::to_string(detections[i].id);
      detection_ros2_msg.bbox.center.x = detections[i].center_x;
      detection_ros2_msg.bbox.center.y = detections[i].center_y;
      detection_ros2_msg.bbox.size_x = detections[i].width_half;
      detection_ros2_msg.bbox.size_y = detections[i].height_half;

      detections_ros2_msg.detections.push_back(detection_ros2_msg);
    }
    start_ethernet.push_back(get_time_in_ms());
    this->detections_publisher_->publish(detections_ros2_msg);
    
    //csv file
	if(this->cluster_flag_ == true) 
	{
		img_num++;
	}
		    
	if(img_num ==EXP_NUM) 
	{
		for(int i=0 ; i<EXP_NUM; i++)
		{
			e_inference.push_back(end_inference[i]-start_inference[i]);
			e_preprocess.push_back(end_preprocess[i]-start_preprocess[i]);
			e_postprocess.push_back(end_postprocess[i]-start_postprocess[i]);
			//e_preethernet.push_back(end_preethernet[i]-start_preethernet[i]);
			//e_while.push_back(end_while[i]-start_while[i]);
			//e_while_max.push_back(end_while_max[i]-start_while[i]);
		}
		file_name << "computing_node"<<this->node_index_<<".csv";
		std::ofstream file(file_name.str());

		//file << std::fixed << std::setprecision(6) << "preprocess_start_time," << "preprocess_time(us)," << "preprocess_end_time,"<< "inference_start_time," << "inference_time(us)," << "inference_end_time,"<< "postprocess_start_time," << "postprocess_time(us)," << "postprocess_end_time,"<< "preethernet_start_time," << "preethernet_time(us)," << "preethernet_end_time\t"<< "ethernet_start_time," << "start_while," << "e_while," << "end_while," << "e_while_max," << "end_while_max\n" ;
		
		file << std::fixed << std::setprecision(6) << "preprocess_start_time," << "preprocess_time(us)," << "preprocess_end_time,"<< "inference_start_time," << "inference_time(us)," << "inference_end_time,"<< "postprocess_start_time," << "postprocess_time(us)," << "postprocess_end_time,"<< "ethernet_start_time\n" ;
			   
		for (int i=0;i<EXP_NUM;i++)
		{
			//file << start_preprocess[i] <<","<< e_preprocess[i] << ","<< end_preprocess[i] << "," << start_inference[i] <<","<< e_inference[i] << ","<< end_inference[i] << ","<< start_postprocess[i] <<","<< e_postprocess[i] << ","<< end_postprocess[i] << ","<< start_preethernet[i] <<","<< e_preethernet[i] << ","<< end_preethernet[i] << ","<< start_ethernet[i] << "," << start_while[i] << ","<< e_while[i] <<","<< end_while[i] << ","<< e_while_max[i] << ","<< end_while_max[i] << "\n";
			file << start_preprocess[i] <<","<< e_preprocess[i] << ","<< end_preprocess[i] << "," << start_inference[i] <<","<< e_inference[i] << ","<< end_inference[i] << ","<< start_postprocess[i] <<","<< e_postprocess[i] << ","<< end_postprocess[i] << ","<< start_ethernet[i] << "," << "\n";
		}
			
		file.close();
			
		std::cerr << "write result at " << "./computing_node"<<this->node_index_<<".csv" << std::endl ;

		exit(0);
		    	
	}
  }
  
  
  else
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Function frameToImage returned 0. No image published.");
  }

  RCLCPP_INFO(this->get_logger(), "======================================================================================================\n");
}

}  // namespace avt_vimba_camera
