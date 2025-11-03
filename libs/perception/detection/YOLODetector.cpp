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

#ifdef HAVE_OPENCV
std::vector<Detection> YOLODetector::postprocessOutput(const std::vector<cv::Mat>& outputs,
                                                      [[maybe_unused]] int originalWidth,
                                                      [[maybe_unused]] int originalHeight) {
  std::vector<Detection> detections;
  
  try {
    // Process outputs passed from forward pass
    if (outputs.empty()) {
      return detections;
    }
    
    // Process each output
    for (size_t outIdx = 0; outIdx < outputs.size(); ++outIdx) {
      cv::Mat output = outputs[outIdx];
      
      // Check if output is valid
      if (output.empty() || output.data == nullptr) {
        continue;  // Skip invalid output
      }
      // Handle different YOLO output formats
      int numClasses = 80;
      
      // Check output dimensions safely
      if (output.dims == 3) {
        // YOLOv8 format: [batch=1, 84, num_anchors] = [1, 84, 8400]
        // 84 = 4 (bbox: x, y, w, h) + 80 (class probabilities)
        
        // Get dimensions safely - check if size array is valid
        if (output.size.p == nullptr || output.dims < 3) {
          continue;  // Skip invalid output
        }
        
        int batch = output.size.p[0];
        int channels = output.size.p[1];  // Should be 84
        int numDetections = output.size.p[2];  // Number of anchors (typically 8400)
        
        if (batch == 1 && channels == 84 && numDetections > 0) {
          // Access 3D tensor directly: [1, 84, num_anchors]
          // Layout is row-major: data[channel * numDetections + detection]
          const float* dataPtr = reinterpret_cast<const float*>(output.data);
          
          // For YOLOv8, data is stored as [batch][channel][detection]
          // Step sizes: step[0] = channels * numDetections, step[1] = numDetections, step[2] = 1
          // Access: data[b * step[0] + c * step[1] + d * step[2]]
          // Since batch=1, we skip batch dimension: data[c * numDetections + d]
          
          for (int i = 0; i < numDetections; ++i) {
            // Get bbox coordinates (channels 0-3)
            float centerX = dataPtr[0 * numDetections + i];  // x
            float centerY = dataPtr[1 * numDetections + i];  // y
            float width = dataPtr[2 * numDetections + i];    // w
            float height = dataPtr[3 * numDetections + i];   // h
            
            // Find class with highest probability (channels 4-83)
            int classId = -1;
            float maxClassProb = 0.0f;
            for (int j = 0; j < numClasses; ++j) {
              float classProb = dataPtr[(4 + j) * numDetections + i];
              if (classProb > maxClassProb) {
                maxClassProb = classProb;
                classId = j;
              }
            }
            
            // In YOLOv8, confidence is already included in class probabilities
            float finalConfidence = maxClassProb;
          
          // Filter by confidence threshold
          if (finalConfidence >= confidenceThreshold_ && classId >= 0) {
            // YOLOv8 outputs are already in image coordinates (640x640 space)
            // Scale coordinates back to original image size
            float scaleX = static_cast<float>(originalWidth) / inputSize_;
            float scaleY = static_cast<float>(originalHeight) / inputSize_;
            
            centerX *= scaleX;  // Scale x to original image
            centerY *= scaleY;  // Scale y to original image
            width *= scaleX;    // Scale width to original image
            height *= scaleY;   // Scale height to original image
            
            // Validate bounding box dimensions
            float minBoxArea = 0.001f * originalWidth * originalHeight;  // At least 0.1% of image
            float maxBoxArea = 0.9f * originalWidth * originalHeight;     // At most 90% of image
            float boxArea = width * height;
            
            // Filter out invalid boxes
            if (width <= 0 || height <= 0 || 
                boxArea < minBoxArea || boxArea > maxBoxArea ||
                centerX < -width/2 || centerX > originalWidth + width/2 ||
                centerY < -height/2 || centerY > originalHeight + height/2) {
              continue;  // Skip invalid detection
            }
            
            // Clamp bounding box to image bounds
            float left = std::max(0.0f, centerX - width / 2.0f);
            float top = std::max(0.0f, centerY - height / 2.0f);
            float right = std::min(static_cast<float>(originalWidth), centerX + width / 2.0f);
            float bottom = std::min(static_cast<float>(originalHeight), centerY + height / 2.0f);
            
            width = right - left;
            height = bottom - top;
            centerX = (left + right) / 2.0f;
            centerY = (top + bottom) / 2.0f;
            
            // Re-check area after clamping
            boxArea = width * height;
            if (boxArea < minBoxArea) {
              continue;  // Skip if too small after clamping
            }
            
            // Create bounding box
            BoundingBox bbox(centerX, centerY, width, height);
            
            // Get class name
            std::string className = (classId >= 0 && static_cast<size_t>(classId) < pImpl_->classNames_.size()) 
                                   ? pImpl_->classNames_[classId] 
                                   : "unknown";
            
            // Create detection
            Detection detection(bbox, finalConfidence, classId, className);
            detections.push_back(detection);
          }
        }
        }  // Close if (batch == 1 && channels == 84 && numDetections > 0)
      } else if (output.dims == 2) {
        // YOLOv5 format: [num_detections, 85] - legacy support
        for (int i = 0; i < output.rows; ++i) {
          const float* data = output.ptr<float>(i);
          
          // Extract confidence and class probabilities
          float confidence = data[4];
          
          // Find class with highest probability
          int classId = -1;
          float maxClassProb = 0.0f;
          for (int j = 5; j < 85; ++j) {
            if (data[j] > maxClassProb) {
              maxClassProb = data[j];
              classId = j - 5;
            }
          }
          
          // Calculate final confidence
          float finalConfidence = confidence * maxClassProb;
          
          if (finalConfidence >= confidenceThreshold_ && classId >= 0) {
            // Extract bounding box coordinates (center_x, center_y, width, height)
            float centerX = data[0];
            float centerY = data[1];
            float width = data[2];
            float height = data[3];
            
            // Scale coordinates back to original image size
            float scaleX = static_cast<float>(originalWidth) / inputSize_;
            float scaleY = static_cast<float>(originalHeight) / inputSize_;
            
            centerX *= scaleX;
            centerY *= scaleY;
            width *= scaleX;
            height *= scaleY;
            
            // Validate bounding box dimensions
            float minBoxArea = 0.001f * originalWidth * originalHeight;
            float maxBoxArea = 0.9f * originalWidth * originalHeight;
            float boxArea = width * height;
            
            if (width <= 0 || height <= 0 || 
                boxArea < minBoxArea || boxArea > maxBoxArea ||
                centerX < -width/2 || centerX > originalWidth + width/2 ||
                centerY < -height/2 || centerY > originalHeight + height/2) {
              continue;
            }
            
            // Clamp bounding box to image bounds
            float left = std::max(0.0f, centerX - width / 2.0f);
            float top = std::max(0.0f, centerY - height / 2.0f);
            float right = std::min(static_cast<float>(originalWidth), centerX + width / 2.0f);
            float bottom = std::min(static_cast<float>(originalHeight), centerY + height / 2.0f);
            
            width = right - left;
            height = bottom - top;
            centerX = (left + right) / 2.0f;
            centerY = (top + bottom) / 2.0f;
            
            boxArea = width * height;
            if (boxArea < minBoxArea) {
              continue;
            }
            
            BoundingBox bbox(centerX, centerY, width, height);
            std::string className = (classId >= 0 && static_cast<size_t>(classId) < pImpl_->classNames_.size()) 
                                   ? pImpl_->classNames_[classId] 
                                   : "unknown";
            
            Detection detection(bbox, finalConfidence, classId, className);
            detections.push_back(detection);
          }
        }
      }
    }
    
    } catch (const cv::Exception& e) {
      std::cerr << "[YOLODetector] OpenCV postprocessing error: " << e.what() << std::endl;
      std::cerr << "[YOLODetector] Error code: " << e.code << " in " << e.func << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "[YOLODetector] Postprocessing failed: " << e.what() << std::endl;
    }
#endif
  
  return detections;
}

std::vector<Detection> YOLODetector::detect(const unsigned char* frame,
                                           int width,
                                           int height,
                                           int channels) {
  std::vector<Detection> detections;

  if (!initialized_) {
    std::cerr << "[YOLODetector] Detector not initialized" << std::endl;
    return detections;
  }

#ifdef HAVE_OPENCV
  if (pImpl_->realMode_) {
    try {
      // Convert raw data to OpenCV Mat
      cv::Mat image(height, width, CV_8UC3, const_cast<unsigned char*>(frame));
      
      // Preprocess image
      preprocessImage(frame, width, height, channels);
      
      // Run inference with workaround for OpenCV 4.5.4 3D tensor bug
      std::vector<cv::Mat> outputs;
      std::vector<std::string> outLayerNames = pImpl_->net_.getUnconnectedOutLayersNames();
      
      try {
        // Try forward with explicit output layer names
        pImpl_->net_.forward(outputs, outLayerNames);
      } catch (const cv::Exception& e) {
        std::cerr << "[YOLODetector] Forward pass failed: " << e.what() << std::endl;
        std::cerr << "[YOLODetector] This may be an OpenCV 4.5.4 compatibility issue with 3D tensor outputs" << std::endl;
        throw;  // Re-throw to be caught by outer catch
      }
      
      // Post-process results (pass outputs directly to avoid double forward call)
      try {
        detections = postprocessOutput(outputs, width, height);
      } catch (const cv::Exception& e) {
        std::cerr << "[YOLODetector] Postprocessing error (OpenCV 4.5.4 3D tensor issue): " 
                  << e.what() << std::endl;
        std::cerr << "[YOLODetector] Recommendation: Upgrade to OpenCV 4.8+ or use older YOLOv5 model (v6.x)" << std::endl;
        throw;  // Re-throw to be caught by outer catch
      } catch (const std::exception& e) {
        std::cerr << "[YOLODetector] Postprocessing failed: " << e.what() << std::endl;
        throw;
      }
      
      // Apply NMS
      detections = applyNMS(detections);
      
      // Filter for human class only (class ID 0)
      std::vector<Detection> humanDetections;
      for (const auto& det : detections) {
        if (det.getClassId() == 0) {  // "person" class
          humanDetections.push_back(det);
        }
      }
      
      return humanDetections;
      
    } catch (const std::exception& e) {
      std::cerr << "[YOLODetector] Detection failed: " << e.what() << std::endl;
      // Fall through to mock mode
    }
  }
#endif
  // Fallback to mock implementation
  // NOTE: Mock mode should only be used for testing when no real model is available
  // In production, please use a proper ONNX YOLO model
  
  // For mock mode, return empty detections to avoid false positives
  // Real YOLO model should be used for actual detection
  static bool mockWarningShown = false;
  if (!mockWarningShown) {
    std::cerr << "[YOLODetector] WARNING: Using mock mode - no detections will be returned" << std::endl;
    std::cerr << "[YOLODetector] Please use a proper ONNX YOLO model for real detection" << std::endl;
    std::cerr << "[YOLODetector] Download yolov8n.onnx and place it in models/ directory" << std::endl;
    mockWarningShown = true;
  }
  
  // Optionally return a single centered detection for testing (commented out)
  // Uncomment only for development/testing purposes:
  /*
  BoundingBox bbox(width * 0.5f, height * 0.5f, width * 0.25f, height * 0.6f);
  Detection det(bbox, 0.90f, 0, "person");
  detections.push_back(det);
  */
  
  return detections;
}

std::vector<Detection> YOLODetector::applyNMS(
    const std::vector<Detection>& detections) {
  std::vector<Detection> filteredDetections;
  
  if (detections.empty()) {
    return filteredDetections;
  }
  
#ifdef HAVE_OPENCV
  try {
    // Convert detections to OpenCV format for NMS
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> classIds;
    
    for (const auto& det : detections) {
      const auto& bbox = det.getBoundingBox();
      
      // Validate bounding box before adding
      float boxWidth = bbox.getWidth();
      float boxHeight = bbox.getHeight();
      
      // Skip boxes that are invalid
      if (boxWidth <= 0 || boxHeight <= 0) {
        continue;
      }
      
      // Convert center-based bbox to corner-based for OpenCV
      int left = static_cast<int>(std::max(0.0f, bbox.getLeft()));
      int top = static_cast<int>(std::max(0.0f, bbox.getTop()));
      int width = static_cast<int>(boxWidth);
      int height = static_cast<int>(boxHeight);
      
      // Skip if box is too small
      if (width < 10 || height < 10) {
        continue;
      }
      
      cv::Rect box(left, top, width, height);
      
      boxes.push_back(box);
      confidences.push_back(det.getConfidence());
      classIds.push_back(det.getClassId());
    }
    
    if (boxes.empty()) {
      return filteredDetections;
    }
    
    // Apply NMS with stricter threshold to remove overlapping detections
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidenceThreshold_, nmsThreshold_, indices);
    
    // Create filtered detections
    for (int idx : indices) {
      filteredDetections.push_back(detections[idx]);
    }
    
  } catch (const std::exception& e) {
    std::cerr << "[YOLODetector] NMS failed: " << e.what() << std::endl;
    // Return empty if NMS fails to avoid false positives
    return filteredDetections;
  }
#else
  // Fallback: simple IoU-based filtering to remove overlaps
  for (size_t i = 0; i < detections.size(); ++i) {
    bool shouldKeep = true;
    
    // Check if this detection overlaps significantly with a higher-confidence one
    for (size_t j = 0; j < detections.size(); ++j) {
      if (i == j) continue;
      
      if (detections[j].getConfidence() > detections[i].getConfidence()) {
        float iou = detections[i].getBoundingBox().computeIoU(
            detections[j].getBoundingBox());
        
        // If IoU is high, remove the lower-confidence detection
        if (iou > nmsThreshold_) {
          shouldKeep = false;
          break;
        }
      }
    }
    
    if (shouldKeep && detections[i].getConfidence() >= confidenceThreshold_) {
      filteredDetections.push_back(detections[i]);
    }
  }
#endif
  
  return filteredDetections;
}

}  // namespace detection
}  // namespace perception

