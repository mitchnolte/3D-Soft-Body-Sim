#ifndef CAMERA_H
#define CAMERA_H

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>


class Camera {
  glm::vec3 position;     // Camera position
  glm::vec3 direction;    // Direction camera is looking
  glm::mat4 projection;   // Projection matrix
  float fov;              // Field of view

public:
  Camera();
  Camera(const glm::vec3& position, const glm::vec3& direction, float aspectRatio, float fov);
  void setPosition(const glm::vec3& position);
  void setDirection(const glm::vec3& direction);
  void setPerspective(float aspectRatio, float fov=0.0);
  glm::mat4 getViewPerspective() const; 

  friend class CameraController;
};


class CameraController {
  Camera* camera;
  glm::vec3 linearVelocity;
  glm::vec2 angularVelocity;
  float moveSpeed;
  float rotateSpeed;
  float dt;

public:
  CameraController(float dt=1, float moveSpeed=6, float rotateSpeed=3);
  void setTimeStep(float dt);
  void bindCamera(Camera& camera);
  void moveCamera(const glm::vec3& direction);
  void rotateCamera(const glm::vec3& direction);
  void updateCamera();
};

#endif
