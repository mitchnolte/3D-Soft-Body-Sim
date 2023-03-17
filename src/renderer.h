#ifndef RENDERER_H
#define RENDERER_H

#include <unordered_map>
#include "camera.h"
#include "mesh.h"
#include "simulation.h"
#include "soft_body.h"


class Renderer {
  GLuint program;
  Camera camera;
  std::unordered_map<std::shared_ptr<SoftBody>, Mesh> meshes;

public:
  Renderer();
  void initializeCamera(const glm::vec3& position, const glm::vec3& direction, float aspectRatio=1,
                        float fov=1.57);
  void display(Simulation& sim);
  void updateMesh(std::shared_ptr<SoftBody> body);
};

#endif
