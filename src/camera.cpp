#include "camera.h"
#include <glm/gtc/matrix_transform.hpp>


Camera::Camera() {}

Camera::Camera(const glm::vec3& position, const glm::vec3& direction, float aspectRatio, float fov)
{
  this->position = position;
  this->direction = glm::normalize(direction);
  this->fov = fov;
  projection = glm::perspective(fov, aspectRatio, 1.0f, 800.0f);
}

void Camera::setPosition(const glm::vec3& position) {
  this->position = position;
}

void Camera::setDirection(const glm::vec3& direction) {
  this->direction = direction;
}

void Camera::setPerspective(float aspectRatio, float fov) {
  if(fov != 0) {
    this->fov = fov;
  }

  projection = glm::perspective(this->fov, aspectRatio, 1.0f, 800.0f);
}

glm::mat4 Camera::getViewPerspective() const {
  return projection * glm::lookAt(position, position+direction, glm::vec3(0.0f, 0.0f, 1.0f));
}
