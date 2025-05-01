#include "imageselection/imageselection.hpp"
#include <rclcpp/rclcpp.hpp>


ImageSelection::ImageSelection(int node_index, int number_of_nodes, double local_fps, double margin) : number_of_nodes_(number_of_nodes), local_fps_(local_fps), node_index_(node_index), margin_(margin)
{
  this->estimated_timestamp_ = -1;
  this->is_compute_node = false;
  double cluster_fps_ = local_fps_ * number_of_nodes_;
  this->inverse_of_fps_ = 1000. / cluster_fps_;
  this->max_estimated_timestamp = -1;
  this->min_estimated_timestamp = -1;
}

ImageSelection::~ImageSelection()
{
}

double ImageSelection::GetEstimatedTimestamp()
{
  return this->estimated_timestamp_ / 1000.0;
}

void ImageSelection::RegisterBaseTimestamp(double init_timestamp)
{
  this->estimated_timestamp_ = init_timestamp * 1000.0 + (this->node_index_ - 1) * this->inverse_of_fps_;
  this->is_compute_node = true;
  //RCLCPP_INFO(rclcpp::get_logger("ImageSelection"),"RegisterBase: init_ts=%.3f s, est_ts=%.3f ms, idx=%d, inv_fps=%.3f ms",init_timestamp, this->estimated_timestamp_, node_index_, inverse_of_fps_);
}

bool ImageSelection::IsSelfOrder(double timestamp)
{
  if (this->is_compute_node)
  {
    return this->IsInRange(timestamp * 1000.0, this->estimated_timestamp_);
  }
  else
  {
    return true;
  }
}

bool ImageSelection::IsInRange(double timestamp, double estimated_timestamp)
{
  this->max_estimated_timestamp = estimated_timestamp + this->margin_;
  this->min_estimated_timestamp = estimated_timestamp - this->margin_;
  

    
  if ((max_estimated_timestamp < timestamp))
  {
    estimated_timestamp += this->number_of_nodes_ * this->inverse_of_fps_;
    return this->IsInRange(timestamp, estimated_timestamp);
  }

  if ((max_estimated_timestamp >= timestamp) && (timestamp >= min_estimated_timestamp))
  {
    this->estimated_timestamp_ = estimated_timestamp + this->number_of_nodes_ * this->inverse_of_fps_;
    return true;
  }
  else
  {
    return false;
  }
}
