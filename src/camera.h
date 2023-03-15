#ifndef CAMERA_H
#define CAMERA_H

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>


class Camera {
  float fov;              // Field of view
  glm::mat4 projection;   // Projection matrix
  glm::vec3 position;     // Camera position
  glm::vec3 direction;    // Direction camera is looking

public:
  Camera();
  Camera(const glm::vec3& position, const glm::vec3& direction, float aspectRatio, float fov);
  void setPosition(const glm::vec3& position);
  void setDirection(const glm::vec3& direction);
  void setPerspective(float aspectRatio, float fov=0.0);
  glm::mat4 getViewPerspective() const; 
};

#endif
