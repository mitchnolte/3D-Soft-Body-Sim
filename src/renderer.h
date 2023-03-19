#ifndef RENDERER_H
#define RENDERER_H

#include <unordered_map>
#include "camera.h"
#include "mesh.h"
#include "simulation.h"
#include "soft_body.h"


class Renderer {
  GLuint shaderProgram;
  Camera camera;
  std::unordered_map<const SoftBody*, Mesh> meshes;

public:
  Renderer();
  void setProgram(GLuint program);
  void initializeCamera(const glm::vec3& position, const glm::vec3& direction, float aspectRatio=1,
                        float fov=1.57);
  void bindMesh(const Mesh& mesh, const SoftBody* softBody);
  void updateMesh(const SoftBody& body);
  void display(const Simulation& sim);
};

#endif
