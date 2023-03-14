#ifndef MESH_H
#define MESH_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include "soft_body.h"

class Mesh {
  GLuint vbuffer;     // Vertex buffer name
  GLuint ibuffer;     // Index buffer name
  GLuint triangles;   // Number of triangles
  GLuint64 vbytes;    // Size of vertices in bytes
  glm::vec4 colour;   // Colour of mesh

public:
  Mesh();
  virtual void display() = 0;
};


class SoftBodyMesh : public Mesh {
  SoftBody body;
};


class TransformableMesh : public Mesh {
  glm::mat4 transformation;

public:
  void display(const glm::mat4& viewPerspective);
};

#endif
