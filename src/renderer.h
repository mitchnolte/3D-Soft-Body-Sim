#ifndef RENDERER_H
#define RENDERER_H

#include <unordered_map>
#include "camera.h"
#include "mesh.h"
#include "simulation.h"
#include "soft_body.h"


class Renderer {
  Camera camera;
  std::unordered_map<SoftBody*, Mesh> meshes;

public:
  void display(Simulation* sim);
};

#endif
