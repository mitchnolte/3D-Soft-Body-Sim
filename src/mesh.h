#ifndef MESH_H
#define MESH_H

#include <GL/glew.h>
#include <glm/glm.hpp>

class Mesh {
  GLuint vbuffer;     // Vertex buffer name
  GLuint ibuffer;     // Index buffer name
  GLuint triangles;   // Number of triangles
  GLuint64 vbytes;    // Size of vertices in bytes
  glm::vec4 colour;   // Colour of mesh

public:
  Mesh();
  
};

#endif
