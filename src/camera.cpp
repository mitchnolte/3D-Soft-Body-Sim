#include "camera.h"
#include <glm/gtc/matrix_transform.hpp>


Camera::Camera() {}

Camera::Camera(const glm::vec3& position, const glm::vec3& direction, float aspectRatio, float fov)
{
  this->position   = position;
  this->direction  = glm::normalize(direction);
  this->fov        = fov;
  this->projection = glm::perspective(fov, aspectRatio, 1.0f, 800.0f);
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

const glm::vec3& Camera::getPosition() {
  return position;
}

/**
 * @brief Returns the projection matrix multiplied by the view matrix.
 */
glm::mat4 Camera::getViewProjection() const {
  return projection * glm::lookAt(position, position+direction, glm::vec3(0.0f, 0.0f, 1.0f));
}


/*******************************************************************************
 *  CAMERA CONTROLLER
 ******************************************************************************/

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
  moveDirection += direction;
}

void CameraController::rotateCamera(const glm::vec3& direction) {
  rotateDirection += glm::vec2(direction);
}

void CameraController::updateCamera() {

  // Movement
  if(moveDirection != glm::vec3(0)) {
    glm::vec3 direction = glm::vec3(camera->direction.x, camera->direction.y, 0.0);
    glm::mat4 toWorldSpace = glm::inverse(glm::lookAt(glm::vec3(0), direction, glm::vec3(0, 0, 1)));
    glm::vec3 moveDirectionWorld = glm::normalize(glm::mat3(toWorldSpace) * moveDirection);
    camera->position += moveSpeed * moveDirectionWorld;
  }

  // Rotation
  float speed = rotateSpeed;
  if(rotateDirection[0] != 0) {
    if(rotateDirection[1] != 0) speed = sqrt(0.5*speed*speed);

    float angle = speed * rotateDirection[0];
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0), angle, glm::vec3(0.0, 0.0, 1.0));
    camera->direction = glm::normalize(glm::mat3(rotation) * camera->direction);
  }
  if(rotateDirection[1] != 0) {
    float angle = speed * rotateDirection[1];
    glm::vec3 axis = glm::cross(camera->direction, glm::vec3{0.0, 0.0, 1.0});
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0), angle, axis);
    camera->direction = glm::normalize(glm::mat3(rotation) * camera->direction);
  }
}
