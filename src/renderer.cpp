#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <GL/glew.h>
#include "renderer.h"
#include "soft_body.h"


void Renderer::display(Simulation* sim) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // TEMP COMMENT: Set uniform variables that apply to everything (light, camera, etc.)
  // TEMP COMMENT: Update mesh vertices from corresponding object's surface masses
  const std::vector<SoftBody>& bodies = sim->getBodies();

  // Display objects
  glm::mat4 viewPerspective = camera.getViewPerspective();
  // for(Mesh mesh : meshes) {
  //   mesh.display(viewPerspective);
  // }
}
