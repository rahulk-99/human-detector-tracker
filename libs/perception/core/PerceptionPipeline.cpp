/**
 * @file PerceptionPipeline.cpp
 * @brief Implementation of PerceptionPipeline class
 * @author Acme Robotics
 * 
 * Phase 1: Enhanced implementation with OpenCV visualization and webcam support
 */

#include "perception/core/PerceptionPipeline.hpp"
#include <iostream>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace perception {
namespace core {

PerceptionPipeline::PerceptionPipeline(
    std::shared_ptr<detection::IDetector> detector,
    std::shared_ptr<tracking::ITracker> tracker,
    std::shared_ptr<ICoordinateTransform> transformer,
    const CameraModel& cameraModel)
    : detector_(detector),
      tracker_(tracker),
      transformer_(transformer),
      cameraModel_(cameraModel),
      frameCount_(0),
      visualizationEnabled_(false) {
  std::cout << "[PerceptionPipeline] Initialized" << std::endl;
#ifdef HAVE_OPENCV
  std::cout << "[PerceptionPipeline] OpenCV support enabled" << std::endl;
#endif
}

PerceptionOutput PerceptionPipeline::processFrame(const unsigned char* frame,
                                                 int width,
                                                 int height,
                                                 int channels,
                                                 double timestamp) {
  PerceptionOutput output;
  output.timestamp = timestamp;
  output.frameCount = ++frameCount_;
  output.success = false;

  try {
    // Step 1: Detect humans
    std::vector<detection::Detection> detections =
        detector_->detect(frame, width, height, channels);

    // Step 2: Transform detections to 3D positions in robot frame
    for (auto& det : detections) {
      // Get 3D position for this detection
      transformer_->imageToRobotFrame(det.getBoundingBox(), width, height);
    }

    // Step 3: Update tracker with detections
    tracker_->update(detections, timestamp);

    // Step 4: Get active tracks
    output.tracks = tracker_->getActiveTracks();

    // Store detections for visualization
    output.detections = detections;
    output.success = true;

    // Visualize if enabled
    if (visualizationEnabled_) {
      visualizeFrame(frame, width, height, output);
    }

  } catch (const std::exception& e) {
    std::cerr << "[PerceptionPipeline] Error processing frame: " << e.what()
              << std::endl;
  }

  return output;
}

std::vector<PerceptionOutput> PerceptionPipeline::processVideo(
    const std::string& videoPath) {
  std::vector<PerceptionOutput> results;
  
  std::cout << "[PerceptionPipeline] Processing video: " << videoPath
            << std::endl;

#ifdef HAVE_OPENCV
  try {
    // Open video file
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
      std::cerr << "[PerceptionPipeline] Error: Could not open video '" << videoPath << "'" << std::endl;
      return results;
    }

    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;

    std::cout << "[PerceptionPipeline] Video opened: " << width << "x" << height
              << " @ " << fps << " FPS" << std::endl;

    // Enable visualization for video playback
    bool originalVizState = visualizationEnabled_;
    visualizationEnabled_ = true;

    cv::Mat frame;
    int frameNumber = 0;

    while (true) {
      if (!cap.read(frame)) {
        break;  // End of video
      }

      double timestamp = frameNumber / std::max(1.0, fps);

      std::vector<unsigned char> frameData(frame.data, frame.data + frame.total() * frame.channels());
      PerceptionOutput output = processFrame(
          frameData.data(), frame.cols, frame.rows, frame.channels(), timestamp);
      results.push_back(output);

      // Allow user to quit early
      char key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 'Q' || key == 27) {
        std::cout << "[PerceptionPipeline] User requested exit" << std::endl;
        break;
      }

      frameNumber++;
    }

    // Restore visualization state
    visualizationEnabled_ = originalVizState;
    cap.release();
    cv::destroyAllWindows();

    std::cout << "[PerceptionPipeline] Video processing complete. Processed "
              << results.size() << " frames" << std::endl;

  } catch (const cv::Exception& e) {
    std::cerr << "[PerceptionPipeline] OpenCV error: " << e.what() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "[PerceptionPipeline] Error processing video: " << e.what() << std::endl;
  }
#else
  // LCOV_EXCL_START - OpenCV not available path, untestable when HAVE_OPENCV is defined
  std::cerr << "[PerceptionPipeline] OpenCV not available - cannot process video" << std::endl;
  // LCOV_EXCL_STOP
#endif

  return results;
}

void PerceptionPipeline::reset() {
  tracker_->reset();
  frameCount_ = 0;
  std::cout << "[PerceptionPipeline] Reset" << std::endl;
}

std::shared_ptr<detection::IDetector> PerceptionPipeline::getDetector() const {
  return detector_;
}

std::shared_ptr<tracking::ITracker> PerceptionPipeline::getTracker() const {
  return tracker_;
}

std::shared_ptr<ICoordinateTransform> PerceptionPipeline::getTransformer() const {
  return transformer_;
}

const CameraModel& PerceptionPipeline::getCameraModel() const {
  return cameraModel_;
}

int PerceptionPipeline::getFrameCount() const {
  return frameCount_;
}

void PerceptionPipeline::setVisualization(bool enable) {
  visualizationEnabled_ = enable;
}

bool PerceptionPipeline::isVisualizationEnabled() const {
  return visualizationEnabled_;
}

void PerceptionPipeline::visualizeFrame(const unsigned char* frame,
                                       int width,
                                       int height,
                                       const PerceptionOutput& output) {
#ifdef HAVE_OPENCV
  try {
    // Convert raw frame data to OpenCV Mat
    cv::Mat image(height, width, CV_8UC3, const_cast<unsigned char*>(frame));
    cv::Mat displayFrame = image.clone();

    // Draw detections (bounding boxes from detector)
    for (const auto& det : output.detections) {
      const auto& bbox = det.getBoundingBox();
      
      // Get bounding box coordinates
      int x1 = static_cast<int>(bbox.getLeft());
      int y1 = static_cast<int>(bbox.getTop());
      int x2 = static_cast<int>(bbox.getRight());
      int y2 = static_cast<int>(bbox.getBottom());

      // Clamp coordinates to image bounds
      x1 = std::max(0, std::min(x1, width - 1));
      y1 = std::max(0, std::min(y1, height - 1));
      x2 = std::max(0, std::min(x2, width - 1));
      y2 = std::max(0, std::min(y2, height - 1));

      // Draw bounding box (green for detections)
      cv::rectangle(displayFrame, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(0, 255, 0), 2);

      // Draw label with confidence
      std::string label = det.getClassName() + " " + 
                         std::to_string(static_cast<int>(det.getConfidence() * 100)) + "%";
      
      int baseline = 0;
      cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      
      // Draw label background
      cv::rectangle(displayFrame,
                    cv::Point(x1, y1 - textSize.height - 10),
                    cv::Point(x1 + textSize.width, y1),
                    cv::Scalar(0, 255, 0), -1);
      
      // Draw label text
      cv::putText(displayFrame, label,
                  cv::Point(x1, y1 - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 0, 0), 1);
    }

    // Draw tracked objects (blue for confirmed tracks, yellow for tentative)
    for (const auto& track : output.tracks) {
      const auto& bbox = track.getBoundingBox();
      
      int x1 = static_cast<int>(bbox.getLeft());
      int y1 = static_cast<int>(bbox.getTop());
      int x2 = static_cast<int>(bbox.getRight());
      int y2 = static_cast<int>(bbox.getBottom());

      // Clamp coordinates
      x1 = std::max(0, std::min(x1, width - 1));
      y1 = std::max(0, std::min(y1, height - 1));
      x2 = std::max(0, std::min(x2, width - 1));
      y2 = std::max(0, std::min(y2, height - 1));

      // Choose color based on track state
      cv::Scalar trackColor;
      std::string stateStr;
      if (track.getState() == tracking::TrackState::CONFIRMED) {
        trackColor = cv::Scalar(255, 0, 0);  // Blue for confirmed
        stateStr = "CONF";
      } else if (track.getState() == tracking::TrackState::TENTATIVE) {
        trackColor = cv::Scalar(0, 255, 255);  // Yellow for tentative
        stateStr = "TENT";
      } else {
        trackColor = cv::Scalar(0, 0, 255);  // Red for lost
        stateStr = "LOST";
      }

      // Draw track bounding box (thicker line)
      cv::rectangle(displayFrame, cv::Point(x1, y1), cv::Point(x2, y2),
                    trackColor, 3);

      // Draw track ID and state
      std::string trackLabel = "ID:" + std::to_string(track.getId()) + " " + stateStr;
      
      int baseline = 0;
      cv::Size textSize = cv::getTextSize(trackLabel, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
      
      // Draw label background
      cv::rectangle(displayFrame,
                    cv::Point(x1, y1 - textSize.height - 5),
                    cv::Point(x1 + textSize.width, y1),
                    trackColor, -1);
      
      // Draw track label text
      cv::putText(displayFrame, trackLabel,
                  cv::Point(x1, y1 - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(255, 255, 255), 2);

      // Draw 3D position info (small text at bottom of box)
      const auto& pos = track.getPosition();
      std::string posStr = "(" + std::to_string(static_cast<int>(pos.getX() * 10) / 10.0f) + "," +
                          std::to_string(static_cast<int>(pos.getZ() * 10) / 10.0f) + "m)";
      
      cv::putText(displayFrame, posStr,
                  cv::Point(x1, y2 + 15),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4,
                  trackColor, 1);
    }

    // Draw frame info
    std::string infoText = "Frame: " + std::to_string(output.frameCount) + 
                          " | Tracks: " + std::to_string(output.tracks.size()) +
                          " | Detections: " + std::to_string(output.detections.size());
    
    cv::putText(displayFrame, infoText,
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 2);

    // Draw instructions
    std::string instructionText = "Press 'q' or ESC to quit";
    cv::putText(displayFrame, instructionText,
                cv::Point(10, height - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);

    // Display frame
    cv::imshow("Human Detection & Tracking", displayFrame);

  } catch (const cv::Exception& e) {
    std::cerr << "[PerceptionPipeline] OpenCV visualization error: " << e.what() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "[PerceptionPipeline] Visualization error: " << e.what() << std::endl;
  }
#else
  // LCOV_EXCL_START - OpenCV not available path, untestable when HAVE_OPENCV is defined
  std::cout << "[PerceptionPipeline] Visualization requested but OpenCV not available" << std::endl;
  // LCOV_EXCL_STOP
#endif
}

}  // namespace core
}  // namespace perception

