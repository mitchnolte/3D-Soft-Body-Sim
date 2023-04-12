#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>
#include "camera.h"


Camera::Camera() {}

Camera::Camera(const glm::vec3& position, const glm::vec3& direction, float aspectRatio, float fov)
{
  this->position   = position;
  this->direction  = glm::normalize(direction);
  this->fov        = fov;
  this->projection = glm::perspective(fov, aspectRatio, 1.0f, 800.0f);
}

void Camera::setPosition(const glm::vec3& position)   { this->position = position;   }
void Camera::setDirection(const glm::vec3& direction) { this->direction = direction; }
const glm::vec3& Camera::getPosition()                { return position;             }


/**
 * @brief  Updates the perspective projection with the given parameters.
 *
 * @param  aspectRatio  Aspect ratio.
 * @param  fov          Field of view.
 */
void Camera::setPerspective(float aspectRatio, float fov) {
  if(fov != 0) {
    this->fov = fov;
  }

  projection = glm::perspective(this->fov, aspectRatio, 1.0f, 800.0f);
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
  this->dt = dt;
}

void CameraController::bindCamera(Camera& camera)             { this->camera = &camera;            }
void CameraController::moveCamera(const glm::vec3& direction) { moveDirection += direction;        }
void CameraController::rotateCamera(double dx, double dy)  { rotateDirection -= glm::vec2(dx, dy); }


/**
 * @brief Updates the position and direction of the camera with one frame worth
 *        of movement and rotation.
 */
void CameraController::updateCamera() {

  // Movement
  if(moveDirection != glm::vec3(0)) {
    glm::vec3 direction = glm::vec3(camera->direction.x, camera->direction.y, 0.0);
    glm::mat4 toWorldSpace = glm::inverse(glm::lookAt(glm::vec3(0), direction, glm::vec3{0, 0, 1}));
    glm::vec3 moveDirectionWorld = glm::normalize(glm::mat3(toWorldSpace) * moveDirection);
    camera->position += moveSpeed * moveDirectionWorld;
  }

  // Horizontal rotation
  if(rotateDirection[0] != 0) {
    float     angle    = rotateSpeed * rotateDirection[0];
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0), angle, glm::vec3{0.0, 0.0, 1.0});
    camera->direction  = glm::normalize(glm::mat3(rotation) * camera->direction);
    rotateDirection[0] = 0;
  }

  // Vertical rotation
  if(rotateDirection[1] != 0) {
    float     angle        = rotateSpeed * rotateDirection[1];
    glm::vec3 axis         = glm::cross(camera->direction, glm::vec3{0.0, 0.0, 1.0});
    glm::mat4 rotation     = glm::rotate(glm::mat4(1.0), angle, axis);
    glm::vec3 newDirection = glm::normalize(glm::mat3(rotation) * camera->direction);

    // Limit rotation
    glm::vec3 xyDirection = glm::normalize(glm::vec3(camera->direction.x, camera->direction.y, 0.0));
    float dot = glm::dot(newDirection, xyDirection);
    if(dot <= 0.05)
      camera->direction = glm::normalize(newDirection + (0.05f-dot)*xyDirection);
    else
      camera->direction = newDirection;
    rotateDirection[1] = 0;
  }
}
