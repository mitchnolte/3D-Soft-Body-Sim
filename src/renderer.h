#ifndef RENDERER_H
#define RENDERER_H

#include <unordered_map>
#include <functional>
#include "camera.h"
#include "mesh.h"
#include "simulation.h"
#include "soft_body.h"

typedef std::function<void (glm::vec3)> KeyFn;
typedef std::unordered_map<int, std::pair<KeyFn, KeyFn>> KeyBindings;


/**
 * Uniform block for a light source.
 */
struct Light {
  GLfloat position[4];
  GLfloat colour[4];
};


class Renderer {
  GLuint shaderProgram;
  GLuint lightBuffer;
  std::unordered_map<const SoftBody*, Mesh> meshes;
  Camera camera;
  CameraController camController;
  KeyBindings keyBindings;
  float fps;

  void initializeCamController();

public:
  Renderer(float fps=60);
  void setProgram(GLuint program);
  void setLight(const Light& light);
  void initializeCamera(const glm::vec3& position, const glm::vec3& direction, float aspectRatio=1,
                        float fov=1.57);
  void bindMesh(const Mesh& mesh, const SoftBody* softBody);
  void updateMesh(const SoftBody& body);
  void handleKeyInput(int key, int action);
  void display(const Simulation& sim);
};

#endif
