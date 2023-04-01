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
  std::vector<Mesh*> meshes;
  Camera camera;
  CameraController camController;
  KeyBindings keyBindings;
  float fps;

  void initializeCamController();

public:
  Renderer(float fps=60);
  ~Renderer();
  void setProgram(GLuint program);
  void setViewport(float winWidth, float winHeight, float fov=0.0);
  void setLight(const Light& light);
  void initializeCamera(const glm::vec3& position, const glm::vec3& direction, float fov, 
                        float aspectRatio=1);
  void addMesh(const Mesh& mesh);
  void addMesh(const SoftCubeMesh& mesh);
  void handleKeyInput(int key, int action);
  void display();
};

#endif
