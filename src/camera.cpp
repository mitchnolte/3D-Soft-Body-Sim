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


/*******************************************************************************
 *  CAMERA CONTROLLER
 ******************************************************************************/



#include <iostream>



CameraController::CameraController(float dt, float moveSpeed, float rotateSpeed) {
  this->moveSpeed   = dt * moveSpeed;
  this->rotateSpeed = dt * rotateSpeed;
  this->dt = dt;
}

void CameraController::setTimeStep(float dt) {
  moveSpeed   = dt * moveSpeed/this->dt;
  rotateSpeed = dt * rotateSpeed/this->dt;
}

void CameraController::bindCamera(Camera& camera) {
  this->camera = &camera;
}

void CameraController::moveCamera(const glm::vec3& direction) {
  linearVelocity += moveSpeed * direction;
}

void CameraController::rotateCamera(const glm::vec3& direction) {
  angularVelocity += rotateSpeed * glm::vec2(direction);
}

void CameraController::updateCamera() {

  // Movement
  if(linearVelocity != glm::vec3(0)) {
    glm::vec3 direction = glm::vec3(camera->direction.x, camera->direction.y, 0.0);
    glm::mat4 toWorldSpace = glm::inverse(glm::lookAt(glm::vec3(0), direction, glm::vec3(0, 0, 1)));
    glm::vec3 velocity = glm::normalize(glm::mat3(toWorldSpace) * linearVelocity);
    camera->position += moveSpeed * velocity;
  }

  // Rotation
  if(angularVelocity[0] != 0) {
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0), angularVelocity[0], glm::vec3(0.0, 0.0, 1.0));
    camera->direction = glm::normalize(glm::mat3(rotation) * camera->direction);
  }
  if(angularVelocity[1] != 0) {
    glm::vec3 axis = glm::cross(camera->direction, glm::vec3{0.0, 0.0, 1.0});
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0), angularVelocity[1], axis);
    camera->direction = glm::normalize(glm::mat3(rotation) * camera->direction);
  }
}
