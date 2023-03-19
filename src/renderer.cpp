#define GLFW_DLL
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <GL/glew.h>
#include "renderer.h"
#include "soft_body.h"


Renderer::Renderer(float fps) : fps(fps) {}

void Renderer::initializeCamController() {
  camController.bindCamera(camera);
  camController.setTimeStep(1/fps);

  // Camera movement keys
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

  // Camera rotation keys
  KeyFn rotateRight = std::bind(&CameraController::rotateCamera, &camController, glm::vec3{-1,0,0});
  KeyFn rotateLeft  = std::bind(&CameraController::rotateCamera, &camController, glm::vec3{ 1,0,0});
  KeyFn rotateUp    = std::bind(&CameraController::rotateCamera, &camController, glm::vec3{ 0,1,0});
  KeyFn rotateDown  = std::bind(&CameraController::rotateCamera, &camController, glm::vec3{0,-1,0});

  keyBindings[GLFW_KEY_RIGHT] = std::make_pair(rotateRight, rotateLeft);
  keyBindings[GLFW_KEY_LEFT] = std::make_pair(rotateLeft, rotateRight);
  keyBindings[GLFW_KEY_UP] = std::make_pair(rotateUp, rotateDown);
  keyBindings[GLFW_KEY_DOWN] = std::make_pair(rotateDown, rotateUp);
}

void Renderer::setProgram(GLuint program) {
  this->shaderProgram = program;
}

/**
 * @brief Binds a mesh to a soft body so the mesh is continually updated to
 *        match the state of the body.
 */
void Renderer::bindMesh(const Mesh& mesh, const SoftBody* softBody) {
  meshes.insert({softBody, mesh});
}



void Renderer::initializeCamera(const glm::vec3& position, const glm::vec3& direction,
                                float aspectRatio, float fov)
{
  camera = Camera(position, direction, aspectRatio, fov);
  initializeCamController();
}


/**
 * @brief Update a mesh's vertices from the corresponding object's surface mass
 *        positions.
 */
void Renderer::updateMesh(const SoftBody& body) {
  const std::vector<Mass*>& masses = body.getSurfaceMasses();
  float vertices[masses.size() * 3];
  for(int i=0; i<masses.size(); i++) {
    Vector pos = masses[i]->getPos();
    vertices[i*3]     = pos[0];
    vertices[i*3 + 1] = pos[1];
    vertices[i*3 + 2] = pos[2];
  }

  glBindBuffer(GL_ARRAY_BUFFER, meshes[&body].getVertexBuf());
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), &vertices);
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


/**
 * @brief Render the simulation.
 */
void Renderer::display(const Simulation& sim) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // TEMP COMMENT: Set uniform variables that apply to everything (light, camera, etc.)

  // Display objects
  camController.updateCamera();
  glm::mat4 viewPerspective = camera.getViewPerspective();
  const std::vector<SoftBody>& bodies = sim.getBodies();
  for(const SoftBody& body : bodies) {
    // updateMesh(body);
    meshes[&body].display(shaderProgram, viewPerspective);
  }
}
