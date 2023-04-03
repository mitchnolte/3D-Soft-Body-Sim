#ifndef RENDERER_H
#define RENDERER_H

#include <unordered_map>
#include <functional>
#include <vector>
#include "camera.h"
class Mesh;
class SoftBodyMesh;


typedef std::function<void (glm::vec3)> KeyFn;
typedef std::unordered_map<int, std::pair<KeyFn, KeyFn>> KeyBindings;

/**
 * @brief Uniform block for a light source.
 */
struct Light {
  GLfloat position[4];
  GLfloat colour[4];
};


/**
 * @brief Handles the visualization of the simulation.
 */
class Renderer {
  GLuint shaderProgram;             // Shader program identifier
  GLuint lightBuffer;               // Light uniform block identifier
  std::vector<Mesh*> meshes;        // Meshes to be rendered
  Camera camera;
  CameraController camController;
  KeyBindings keyBindings;          // Binds keycode to key functions
  float fps;                        // Frames per second

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
  void addMesh(const SoftBodyMesh& mesh);
  void handleKeyInput(int key, int action);
  void display();
};

#endif
