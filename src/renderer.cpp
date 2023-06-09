#define GLFW_DLL
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include "renderer.h"
#include "mesh.h"
#include "soft_body.h"


Renderer::Renderer(float fps) {
  this->fps = fps;
}

Renderer::~Renderer() {
  for(int i=0; i<meshes.size(); i++) {
    delete meshes[i];
  }
}

/**
 * @brief Initializes key bindings and other parameters of the camera
 *        controller.
 */
void Renderer::initializeCamController() {
  camController.bindCamera(camera);
  camController.setTimeStep(1/fps);

  // Camera movement keys (WASD)
  KeyFn moveRight   = std::bind(&CameraController::moveCamera, &camController, glm::vec3{ 1, 0, 0});
  KeyFn moveLeft    = std::bind(&CameraController::moveCamera, &camController, glm::vec3{-1, 0, 0});
  KeyFn moveUp      = std::bind(&CameraController::moveCamera, &camController, glm::vec3{ 0, 1, 0});
  KeyFn moveDown    = std::bind(&CameraController::moveCamera, &camController, glm::vec3{ 0,-1, 0});
  KeyFn moveBack    = std::bind(&CameraController::moveCamera, &camController, glm::vec3{ 0, 0, 1});
  KeyFn moveForward = std::bind(&CameraController::moveCamera, &camController, glm::vec3{ 0, 0,-1});

  keyBindings[GLFW_KEY_W] = std::make_pair(moveForward, moveBack);      // Move camera forwards  (W)
  keyBindings[GLFW_KEY_A] = std::make_pair(moveLeft, moveRight);        // Move camera left      (A)
  keyBindings[GLFW_KEY_S] = std::make_pair(moveBack, moveForward);      // Move camera backwards (S)
  keyBindings[GLFW_KEY_D] = std::make_pair(moveRight, moveLeft);        // Move camera right     (D)
  keyBindings[GLFW_KEY_SPACE] = std::make_pair(moveUp, moveDown);       // Move camera up    (SPACE)
  keyBindings[GLFW_KEY_LEFT_SHIFT] = std::make_pair(moveDown, moveUp);  // Move camera down  (SHIFT)
}


void Renderer::setProgram(GLuint program) {
  this->shaderProgram = program;
  glUseProgram(program);
  glGenBuffers(1, &lightBuffer);
}

void Renderer::setViewport(float winWidth, float winHeight, float fov) {
  glViewport(0, 0, winWidth, winHeight);
  camera.setPerspective(winWidth/winHeight, fov);
}

void Renderer::setLight(const Light& light) {
	glBindBuffer(GL_UNIFORM_BUFFER, lightBuffer);
	glBufferData(GL_UNIFORM_BUFFER, sizeof(light), &light, GL_STATIC_DRAW);
}

void Renderer::addMesh(const Mesh& mesh) {
  meshes.push_back(new Mesh(mesh));
}

void Renderer::addMesh(const SoftBodyMesh& mesh) {
  meshes.push_back(new SoftBodyMesh(mesh));
}


void Renderer::initializeCamera(const glm::vec3& position, const glm::vec3& direction, float fov,
                                float aspectRatio)
{
  camera = Camera(position, direction, aspectRatio, fov);
  initializeCamController();
}


void Renderer::handleKeyInput(int key, int action) {
  if(action == GLFW_PRESS) {
    if(keyBindings.count(key) > 0) {
      keyBindings[key].first(glm::vec3());
    }
  } 
  else if(action == GLFW_RELEASE) {
    if(keyBindings.count(key) > 0) {
      keyBindings[key].second(glm::vec3());
    }
  }
}


void Renderer::handleMouseInput(double dx, double dy) {
  camController.rotateCamera(dx, dy);
}


/**
 * @brief Renders the simulation.
 */
void Renderer::display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Set uniform variables
	glBindBufferBase(GL_UNIFORM_BUFFER, 1, lightBuffer);
	int camPos = glGetUniformLocation(shaderProgram, "camPos");
	glUniform3fv(camPos, 1, glm::value_ptr(camera.getPosition()));

  // Display objects
  camController.updateCamera();
  glm::mat4 viewProjection = camera.getViewProjection();
  for(Mesh* mesh : meshes) {
    mesh->display(shaderProgram, viewProjection);
  }
}
