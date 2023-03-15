#ifndef MESH_H
#define MESH_H

#include <GL/glew.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include "soft_body.h"


class Mesh {
protected:
  GLuint vertexBuf;
  GLuint indexBuf;
  GLuint numTris;
  GLuint64 vertexSize;  // Size of vertices in bytes
  glm::vec4 colour;

  void loadVertexData(GLuint program);

public:
  Mesh();
  GLuint getVertexBuf();
  GLuint getIndexBuf();
  virtual void display(GLuint program, const glm::mat4& viewPerspective);
};


class TransformableMesh : public Mesh {
  glm::mat4 transformation;

public:
  void display(GLuint program, const glm::mat4& viewPerspective);
};

#endif
