#ifndef CAMERA_H
#define CAMERA_H

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>


/**
 * @brief Controls the view of the simulation.
 */
class Camera {
  glm::vec3 position;     // Camera position in world space
  glm::vec3 direction;    // Direction camera is facing in world space
  glm::mat4 projection;   // Projection matrix
  float fov;              // Field of view

public:
  Camera();
  Camera(const glm::vec3& position, const glm::vec3& direction, float aspectRatio, float fov);
  void setPosition(const glm::vec3& position);
  void setDirection(const glm::vec3& direction);
  void setPerspective(float aspectRatio, float fov=0.0);
  const glm::vec3& getPosition();
  glm::mat4 getViewProjection() const; 

  friend class CameraController;
};


/**
 * @brief Controls the movement and rotation of the camera.
 */
class CameraController {
  Camera* camera;
  glm::vec3 moveDirection;
  glm::vec2 rotateDirection;
  float moveSpeed;
  float rotateSpeed;
  float dt;

public:
  CameraController(float dt=1.0, float moveSpeed=6.0, float rotateSpeed=0.1);
  void setTimeStep(float dt);
  void bindCamera(Camera& camera);
  void moveCamera(const glm::vec3& direction);
  void rotateCamera(double dx, double dy);
  void updateCamera();
};

#endif
