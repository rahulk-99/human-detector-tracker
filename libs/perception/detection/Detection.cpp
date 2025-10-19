/**
 * @file Detection.cpp
 * @brief Implementation of Detection class
 * @author Acme Robotics
 */

#include "perception/detection/Detection.hpp"

namespace perception {
namespace detection {

Detection::Detection()
    : confidence_(0.0f), classId_(-1), className_("unknown") {}

Detection::Detection(const BoundingBox& bbox,
                    float confidence,
                    int classId,
                    const std::string& className)
    : bbox_(bbox),
      confidence_(confidence),
      classId_(classId),
      className_(className) {}

const BoundingBox& Detection::getBoundingBox() const {
  return bbox_;
}

float Detection::getConfidence() const {
  return confidence_;
}

int Detection::getClassId() const {
  return classId_;
}

const std::string& Detection::getClassName() const {
  return className_;
}

void Detection::setBoundingBox(const BoundingBox& bbox) {
  bbox_ = bbox;
}

void Detection::setConfidence(float confidence) {
  confidence_ = confidence;
}

void Detection::setClass(int classId, const std::string& className) {
  classId_ = classId;
  className_ = className;
}

bool Detection::isValid(float threshold) const {
  return confidence_ >= threshold;
}

}  // namespace detection
}  // namespace perception

