#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <GL/glew.h>
#include "renderer.h"
#include "soft_body.h"


Renderer::Renderer() {}

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


/**
 * @brief Render the simulation.
 */
void Renderer::display(const Simulation& sim) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // TEMP COMMENT: Set uniform variables that apply to everything (light, camera, etc.)

  // Display objects
  const std::vector<SoftBody>& bodies = sim.getBodies();
  glm::mat4 viewPerspective = camera.getViewPerspective();
  for(const SoftBody& body : bodies) {
    // updateMesh(body);
    meshes[&body].display(shaderProgram, viewPerspective);
  }
}
