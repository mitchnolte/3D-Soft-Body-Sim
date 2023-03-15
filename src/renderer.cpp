#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <GL/glew.h>
#include "renderer.h"
#include "soft_body.h"


void Renderer::display(Simulation* sim) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  // TEMP COMMENT: Set uniform variables that apply to everything (light, camera, etc.)

  // Display objects
  const std::vector<SoftBody>& bodies = sim->getBodies();
  glm::mat4 viewPerspective = camera.getViewPerspective();
  for(const SoftBody& body : bodies) {
    updateMesh(body);
    meshes[&body].display(viewPerspective);
  }
}


/**
 * @brief Update a mesh's vertices from corresponding object's surface mass
 *        positions.
 * @param bodies Soft bodies in the simulation.
 */
void Renderer::updateMesh(const SoftBody& body) {
  const std::vector<const Mass&>& masses = body.getSurfaceMasses();
  float vertices[masses.size() * 3];
  for(int i=0; i<masses.size(); i++) {
    Vector pos = masses[i].getPos();
    vertices[i*3]     = pos[0];
    vertices[i*3 + 1] = pos[1];
    vertices[i*3 + 2] = pos[2];
  }

  glBindBuffer(GL_ARRAY_BUFFER, meshes[&body].getVbuffer());
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), &vertices);
}
