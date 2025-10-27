/**
 * @file YOLODetector.cpp
 * @brief Implementation of YOLODetector class
 * @author Acme Robotics
 * 
 * Phase 1: Real YOLO implementation with OpenCV DNN
 */

#include "perception/detection/YOLODetector.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdexcept>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#endif

namespace perception {
namespace detection {

// Pimpl implementation for OpenCV DNN
class YOLODetector::Impl {
 public:
#ifdef HAVE_OPENCV
  cv::dnn::Net net_;
  std::vector<std::string> classNames_;
  std::vector<cv::Scalar> colors_;
  bool realMode_;  // Flag to indicate if real YOLO is available
#endif
  
  Impl() {
#ifdef HAVE_OPENCV
    realMode_ = false;  // Start in mock mode
    
    // Initialize COCO class names (80 classes)
    classNames_ = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
        "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
        "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };
    
    // Generate random colors for each class
    colors_.resize(classNames_.size());
    for (size_t i = 0; i < colors_.size(); ++i) {
      colors_[i] = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);
    }
#endif
  }
  
  ~Impl() = default;
};

YOLODetector::YOLODetector(const std::string& modelPath,
                          float confidenceThreshold,
                          float nmsThreshold,
                          int inputSize)
    : modelPath_(modelPath),
      confidenceThreshold_(confidenceThreshold),
      nmsThreshold_(nmsThreshold),
      inputSize_(inputSize),
      initialized_(false),
      pImpl_(std::make_unique<Impl>()) {
  
  // Validate parameters
  if (modelPath.empty()) {
    throw std::invalid_argument("Model path cannot be empty");
  }
  
  if (confidenceThreshold < 0.0f || confidenceThreshold > 1.0f) {
    throw std::invalid_argument("Confidence threshold must be between 0.0 and 1.0");
  }
  
  if (nmsThreshold < 0.0f || nmsThreshold > 1.0f) {
    throw std::invalid_argument("NMS threshold must be between 0.0 and 1.0");
  }
  
  if (inputSize <= 0) {
    throw std::invalid_argument("Input size must be positive");
  }
  
  initialized_ = initialize();
}

YOLODetector::~YOLODetector() = default;

bool YOLODetector::initialize() {
#ifdef HAVE_OPENCV
  try {
    // Check if model file exists
    std::ifstream file(modelPath_);
    if (!file.good()) {
      std::cerr << "[YOLODetector] Model file not found: " << modelPath_ << std::endl;
      std::cout << "[YOLODetector] Falling back to mock implementation" << std::endl;
      return true;  // Allow mock mode
    }
    
    // Load the model
    std::cout << "[YOLODetector] Loading model: " << modelPath_ << std::endl;
    
    bool modelLoaded = false;
    
    // Try to load as ONNX first
    try {
      pImpl_->net_ = cv::dnn::readNetFromONNX(modelPath_);
      std::cout << "[YOLODetector] Loaded ONNX model successfully" << std::endl;
      modelLoaded = true;
    } catch (const cv::Exception& e) {
      std::cout << "[YOLODetector] ONNX loading failed: " << e.what() << std::endl;
    }
    
    // Try PyTorch if ONNX failed
    if (!modelLoaded) {
      try {
        pImpl_->net_ = cv::dnn::readNetFromTorch(modelPath_);
        std::cout << "[YOLODetector] Loaded PyTorch model successfully" << std::endl;
        modelLoaded = true;
      } catch (const cv::Exception& e2) {
        std::cout << "[YOLODetector] PyTorch loading failed: " << e2.what() << std::endl;
      }
    }
    
    if (modelLoaded) {
      // Set backend and target (prefer CPU for compatibility)
      pImpl_->net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      pImpl_->net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      
      pImpl_->realMode_ = true;  // Enable real YOLO mode
      
      std::cout << "[YOLODetector] Model initialized successfully" << std::endl;
      std::cout << "[YOLODetector] Input size: " << inputSize_ << "x" << inputSize_ << std::endl;
      std::cout << "[YOLODetector] Confidence threshold: " << confidenceThreshold_ << std::endl;
      std::cout << "[YOLODetector] NMS threshold: " << nmsThreshold_ << std::endl;
      
      return true;
    } else {
      std::cout << "[YOLODetector] Could not load model - falling back to mock implementation" << std::endl;
      pImpl_->realMode_ = false;  // Stay in mock mode
      return true;  // Allow mock mode
    }
    
  } catch (const std::exception& e) {
    std::cerr << "[YOLODetector] Initialization failed: " << e.what() << std::endl;
    std::cout << "[YOLODetector] Falling back to mock implementation" << std::endl;
    return true;  // Allow mock mode
  }
#else
  std::cout << "[YOLODetector] OpenCV not available - using mock implementation" << std::endl;
  return true;  // Fallback to mock for Phase 0
#endif
}

void YOLODetector::setConfidenceThreshold(float threshold) {
  if (threshold < 0.0f || threshold > 1.0f) {
    throw std::invalid_argument("Confidence threshold must be between 0.0 and 1.0");
  }
  confidenceThreshold_ = threshold;
}

float YOLODetector::getConfidenceThreshold() const {
  return confidenceThreshold_;
}

void YOLODetector::setNmsThreshold(float threshold) {
  if (threshold < 0.0f || threshold > 1.0f) {
    throw std::invalid_argument("NMS threshold must be between 0.0 and 1.0");
  }
  nmsThreshold_ = threshold;
}

float YOLODetector::getNmsThreshold() const {
  return nmsThreshold_;
}

bool YOLODetector::isInitialized() const {
  return initialized_;
}

int YOLODetector::getInputSize() const {
  return inputSize_;
}

void YOLODetector::preprocessImage([[maybe_unused]] const unsigned char* frame,
                                  [[maybe_unused]] int width,
                                  [[maybe_unused]] int height,
                                  [[maybe_unused]] int channels) {
#ifdef HAVE_OPENCV
  try {
    // Convert raw data to OpenCV Mat
    cv::Mat image(height, width, CV_8UC3, const_cast<unsigned char*>(frame));
    
    // Convert BGR to RGB if needed (OpenCV uses BGR by default)
    cv::Mat rgbImage;
    cv::cvtColor(image, rgbImage, cv::COLOR_BGR2RGB);
    
    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(rgbImage, blob, 1.0/255.0, 
                          cv::Size(inputSize_, inputSize_), 
                          cv::Scalar(0, 0, 0), true, false, CV_32F);
    
    // Set input to the network
    pImpl_->net_.setInput(blob);
    
  } catch (const std::exception& e) {
    std::cerr << "[YOLODetector] Preprocessing failed: " << e.what() << std::endl;
  }
#endif
}



}  // namespace detection
}  // namespace perception

