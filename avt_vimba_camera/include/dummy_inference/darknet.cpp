/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

// yolo object detector
#include "darknet_ros/YoloObjectDetector.hpp"

// Check for xServer
#include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros {

char *cfg;
char *weights;
char *data;
char **detectionNames;

YoloObjectDetector::YoloObjectDetector()
    : numClasses_(0),
      classLabels_(0),
      rosBoxes_(0),
      rosBoxCounter_(0),
      action_active_(false),
      preempt_requested_(false)
{
  RCLCPP_INFO(get_logger(), "[YoloObjectDetector] Node started.");

  declare_parameter("image_view.enable_opencv", true);
  declare_parameter("image_view.wait_key_delay", 3);
  declare_parameter("image_view.enable_console_output", false);
  declare_parameter("yolo_model.detection_classes.names", std::vector<std::string>(0));

  declare_parameter("yolo_model.threshold.value", 0.3f);
  declare_parameter("yolo_model.weight_file.name", std::string("yolov2-tiny.weights"));
  declare_parameter("weights_path", std::string("/default"));

  declare_parameter("yolo_model.config_file.name", std::string("yolov2-tiny.cfg"));
  declare_parameter("config_path", std::string("/default"));

  declare_parameter("subscribers.camera_reading.topic", std::string("/camera/image_raw"));
  declare_parameter("subscribers.camera_reading.queue_size", 1);
  declare_parameter("publishers.object_detector.topic", std::string("found_object"));
  declare_parameter("publishers.object_detector.queue_size", 1);
  declare_parameter("publishers.object_detector.latch", false);
  declare_parameter("publishers.bounding_boxes.topic", std::string("bounding_boxes"));
  declare_parameter("publishers.bounding_boxes.queue_size", 1);
  declare_parameter("publishers.bounding_boxes.latch", false);
  declare_parameter("publishers.detection_image.topic", std::string("detection_image"));
  declare_parameter("publishers.detection_image.queue_size", 1);
  declare_parameter("publishers.detection_image.latch", true);
  declare_parameter("yolo_model.window_name", std::string("YOLO"));

  declare_parameter("actions.camera_reading.topic", std::string("check_for_objects"));

  init();
}

YoloObjectDetector::~YoloObjectDetector()
{
  {
    std::unique_lock<std::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

bool YoloObjectDetector::readParameters()
{
  // Load common parameters.
  get_parameter("image_view.enable_opencv", viewImage_);
  get_parameter("image_view.wait_key_delay", waitKeyDelay_);
  get_parameter("image_view.enable_console_output", enableConsoleOutput_);

  get_parameter("yolo_model.window_name", windowName_);

  // Check if Xserver is running on Linux.
  if (XOpenDisplay(NULL)) {
    // Do nothing!
    RCLCPP_INFO(get_logger(), "[YoloObjectDetector] Xserver is running.");
  } else {
    RCLCPP_INFO(get_logger(), "[YoloObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }

  // Set vector sizes.
  get_parameter("yolo_model.detection_classes.names", classLabels_);
  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);

  return true;
}

void YoloObjectDetector::init()
{
  // Read parameters from config file.
  if (!readParameters()) {
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(), "[YoloObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh;
  get_parameter("yolo_model.threshold.value", thresh);

  // Path to weights file.
  get_parameter("yolo_model.weight_file.name", weightsModel);
  get_parameter("weights_path", weightsPath);
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  get_parameter("yolo_model.config_file.name", configModel);
  get_parameter("config_path", configPath);
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_,
                0, 0, 1, 0.5, 0, 0, 0, 0);

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  int cameraQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;
  std::string detectionImageTopicName;
  int detectionImageQueueSize;
  bool detectionImageLatch;

  get_parameter("subscribers.camera_reading.topic", cameraTopicName);
  get_parameter("subscribers.camera_reading.queue_size", cameraQueueSize);
  get_parameter("publishers.object_detector.topic", objectDetectorTopicName);
  get_parameter("publishers.object_detector.queue_size", objectDetectorQueueSize);
  get_parameter("publishers.object_detector.latch", objectDetectorLatch);
  get_parameter("publishers.bounding_boxes.topic", boundingBoxesTopicName);
  get_parameter("publishers.bounding_boxes.queue_size", boundingBoxesQueueSize);
  get_parameter("publishers.bounding_boxes.latch", boundingBoxesLatch);
  get_parameter("publishers.detection_image.topic", detectionImageTopicName);
  get_parameter("publishers.detection_image.queue_size", detectionImageQueueSize);
  get_parameter("publishers.detection_image.latch", detectionImageLatch);
}

void YoloObjectDetector::cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  RCLCPP_DEBUG(get_logger(), "[YoloObjectDetector] USB image received.");

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      std::unique_lock<std::shared_mutex> lockImageCallback(mutexImageCallback_);
      imageHeader_ = msg->header;
      camImageCopy_ = cam_image->image.clone();
    }
    {
      std::unique_lock<std::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }

}

bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage)
{
  if (detectionImagePublisher_->get_subscription_count() < 1)
    return false;
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = this->now();
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = "bgr8";
  cvImage.image = detectionImage;
  detectionImagePublisher_->publish(*cvImage.toImageMsg());
  RCLCPP_DEBUG(get_logger(), "Detection image has been published.");
  return true;
}

int YoloObjectDetector::sizeNetwork(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      count += l.outputs;
    }
  }
  return count;
}

void YoloObjectDetector::rememberNetwork(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}

detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes)
{
  int i, j;
  int count = 0;
  fill_cpu(demoTotal_, 0, avg_, 1);
  for(j = 0; j < demoFrame_; ++j){
    axpy_cpu(demoTotal_, 1./demoFrame_, predictions_[j], 1, avg_, 1);
  }
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
  // detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
  detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes, 1); // letter box
  return dets;
}

void *YoloObjectDetector::detectInThread(cv::Mat& )
{
  running_ = 1;
  float nms = .4;

  layer l = net_->layers[net_->n - 1];
  float *X = buffLetter_[(buffIndex_ + 2) % 3].data;
  float *prediction = network_predict(*net_, X);

  rememberNetwork(net_);
  detection *dets = 0;
  int nboxes = 0;
  dets = avgPredictions(net_, &nboxes);

  if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

  if (enableConsoleOutput_) {
    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nFPS:%.1f : ( %s )\n",fps_,ss_fps.str().c_str());
    printf("Objects:\n\n");
  }


  // image display = buff_[(buffIndex_+2) % 3];
  // draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);
  
  // draw_detections_v3(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_, 1);
  // extract the bounding boxes and send them to ROS
  // int i, j;
  // int count = 0;
  // for (i = 0; i < nboxes; ++i) {
  //   float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
  //   float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
  //   float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
  //   float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

  //   if (xmin < 0)
  //     xmin = 0;
  //   if (ymin < 0)
  //     ymin = 0;
  //   if (xmax > 1)
  //     xmax = 1;
  //   if (ymax > 1)
  //     ymax = 1;

  //   // iterate through possible boxes and collect the bounding boxes
  //   for (j = 0; j < demoClasses_; ++j) {
  //     if (dets[i].prob[j]) {
  //       float x_center = (xmin + xmax) / 2;
  //       float y_center = (ymin + ymax) / 2;
  //       float BoundingBox_width = xmax - xmin;
  //       float BoundingBox_height = ymax - ymin;

  //       // define bounding box
  //       // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
  //       if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
  //         roiBoxes_[count].x = x_center;
  //         roiBoxes_[count].y = y_center;
  //         roiBoxes_[count].w = BoundingBox_width;
  //         roiBoxes_[count].h = BoundingBox_height;
  //         roiBoxes_[count].Class = j;
  //         roiBoxes_[count].prob = dets[i].prob[j];
  //         count++;
  //       }
  //     }
  //   }
  // }

  // // create array to store found bounding boxes
  // // if no object detected, make sure that ROS knows that num = 0
  // if (count == 0) {
  //   roiBoxes_[0].num = 0;
  // } else {
  //   roiBoxes_[0].num = count;
  // }

  free_detections(dets, nboxes);
  demoIndex_ = (demoIndex_ + 1) % demoFrame_;
  running_ = 0;
  return 0;
}

float get_pixel_cp(image m, int x, int y, int c)
{
    assert(x < m.w && y < m.h && c < m.c);
    return m.data[c*m.h*m.w + y*m.w + x];
}

image **load_alphabet_with_file_cp(char *datafile) {
  int i, j;
  const int nsize = 8;
  image **alphabets = (image**)calloc(nsize, sizeof(image));
  char* labels = "/labels/%d_%d.png";
  char * files = (char *) malloc(1 + strlen(datafile)+ strlen(labels) );
  strcpy(files, datafile);
  strcat(files, labels);
  for(j = 0; j < nsize; ++j){
    alphabets[j] = (image*)calloc(128, sizeof(image));
    for(i = 32; i < 127; ++i){
      char buff[256];
      sprintf(buff, files, i, j);
      alphabets[j][i] = load_image_color(buff, 0, 0);
    }
  }
  return alphabets;
}

void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh,
                                      char **names, int classes,
                                      int delay, char *prefix, int avg_frames, float hier, int w, int h,
                                      int frames, int fullscreen)
{
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  image **alphabet = load_alphabet_with_file_cp(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  fullScreen_ = fullscreen;
  printf("YOLO V3\n");
  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
}

void YoloObjectDetector::yolo()
{
  srand(2222222);

  int i;
  demoTotal_ = sizeNetwork(net_);
  predictions_ = (float **) calloc(demoFrame_, sizeof(float*));
  for (i = 0; i < demoFrame_; ++i){
      predictions_[i] = (float *) calloc(demoTotal_, sizeof(float));
  }
  avg_ = (float *) calloc(demoTotal_, sizeof(float));

  layer l = net_->layers[net_->n - 1];
  roiBoxes_ = (darknet_ros::RosBox_ *) calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));

  {
    std::shared_lock<std::shared_mutex> lock(mutexImageCallback_);
    CvMatWithHeader_ imageAndHeader = getCvMatWithHeader();
    buff_[0] = mat_to_image(imageAndHeader.image);
    headerBuff_[0] = imageAndHeader.header;
  }
  buff_[1] = copy_image(buff_[0]);
  buff_[2] = copy_image(buff_[0]);
  headerBuff_[1] = headerBuff_[0];
  headerBuff_[2] = headerBuff_[0];
  buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
  // buff_[0] = mat_to_image(imageAndHeader.image);
  disp_ = image_to_mat(buff_[0]);

  int count = 0;
  if (!demoPrefix_ && viewImage_) {
    cv::namedWindow(windowName_.c_str(), cv::WINDOW_NORMAL);
    if (fullScreen_) {
      cv::setWindowProperty(windowName_.c_str(), cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    } else {
      cv::moveWindow(windowName_.c_str(), 0, 0);
      cv::resizeWindow(windowName_.c_str(), 640, 480);
    }
  }

  demoTime_ = what_time_is_it_now();

  while (!demoDone_) {
    buffIndex_ = (buffIndex_ + 1) % 3;
    fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);
    detect_thread = std::thread(&YoloObjectDetector::detectInThread, this);
    if (!demoPrefix_) {
      fps_ = 1./(what_time_is_it_now() - demoTime_);

      if (start_count > START_COUNT) {
        if (fps_ > max_fps)
          max_fps = fps_;
        else if (fps_ < min_fps)
          min_fps = fps_;
      }
      else
        start_count++;

      ss_fps.str("");
      ss_fps << "MAX:" << max_fps << " MIN:" << min_fps;

      demoTime_ = what_time_is_it_now();
      if (viewImage_) {
        displayInThread(0);
      } else {
        generate_image_cp(buff_[(buffIndex_ + 1)%3], disp_);
      }
      publishInThread();
    } else {
      char name[256];
      sprintf(name, "%s_%08d", demoPrefix_, count);
      save_image(buff_[(buffIndex_ + 1) % 3], name);
    }
    fetch_thread.join();
    detect_thread.join();
    ++count;
    if (!isNodeRunning()) {
      demoDone_ = true;
    }
  }
}

CvMatWithHeader_ YoloObjectDetector::getCvMatWithHeader() {
  CvMatWithHeader_ header = {.image = camImageCopy_, .header = imageHeader_};
  return header;
}

bool YoloObjectDetector::getImageStatus(void)
{
  std::shared_lock<std::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::isNodeRunning(void)
{
  std::shared_lock<std::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

// void *YoloObjectDetector::publishInThread()
// {
//   // Publish image.
//   // cv::Mat cvImage = cv::cvarrToMat(ipl_);
//   cv::Mat cvImage = disp_;
//   if (!publishDetectionImage(cv::Mat(cvImage))) {
//     RCLCPP_DEBUG(get_logger(), "Detection image has not been broadcasted.");
//   }

//   // Publish bounding boxes and detection result.
//   int num = roiBoxes_[0].num;
//   if (num > 0 && num <= 100) {
//     for (int i = 0; i < num; i++) {
//       for (int j = 0; j < numClasses_; j++) {
//         if (roiBoxes_[i].Class == j) {
//           rosBoxes_[j].push_back(roiBoxes_[i]);
//           rosBoxCounter_[j]++;
//         }
//       }
//     }

//     darknet_ros_msgs::msg::ObjectCount msg;
//     msg.header.stamp = this->now();
//     msg.header.frame_id = "detection";
//     msg.count = num;
//     objectPublisher_->publish(msg);

//     for (int i = 0; i < numClasses_; i++) {
//       if (rosBoxCounter_[i] > 0) {
//         darknet_ros_msgs::msg::BoundingBox boundingBox;

//         for (int j = 0; j < rosBoxCounter_[i]; j++) {
//           int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
//           int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
//           int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
//           int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;

//           boundingBox.class_id = classLabels_[i];
//           boundingBox.id = i;
//           boundingBox.probability = rosBoxes_[i][j].prob;
//           boundingBox.xmin = xmin;
//           boundingBox.ymin = ymin;
//           boundingBox.xmax = xmax;
//           boundingBox.ymax = ymax;
//           boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
//         }
//       }
//     }
//     boundingBoxesResults_.header.stamp = this->now();
//     boundingBoxesResults_.header.frame_id = "detection";
//     boundingBoxesResults_.image_header = headerBuff_[(buffIndex_ + 1) % 3];
//     boundingBoxesPublisher_->publish(boundingBoxesResults_);
//   } else {
//     darknet_ros_msgs::msg::ObjectCount msg;
//     msg.header.stamp = this->now();
//     msg.header.frame_id = "detection";
//     msg.count = 0;
//     objectPublisher_->publish(msg);
//   }
//   if (isCheckingForObjects()) {
//     RCLCPP_DEBUG(get_logger(), "[YoloObjectDetector] check for objects in image.");
//     auto result = std::make_shared<CheckForObjectsAction::Result>();

//     result->id = buffId_[0];
//     result->bounding_boxes = boundingBoxesResults_;
//     goal_handle_->succeed(result);
//     action_active_ = false;
//   }
//   boundingBoxesResults_.bounding_boxes.clear();
//   for (int i = 0; i < numClasses_; i++) {
//     rosBoxes_[i].clear();
//     rosBoxCounter_[i] = 0;
//   }

//   return 0;
// }

void YoloObjectDetector::generate_image_cp(image p, cv::Mat& disp) {
  int x, y, k;
  if (p.c == 3) rgbgr_image(p);
  // normalize_image(copy);

  int step = disp.step;
  for (y = 0; y < p.h; ++y) {
    for (x = 0; x < p.w; ++x) {
      for (k = 0; k < p.c; ++k) {
        disp.data[y * step + x * p.c + k] = (unsigned char)(get_pixel_cp(p, x, y, k) * 255);
      }
    }
  }
}

} /* namespace darknet_ros*/
