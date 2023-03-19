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

  void sendVertexData(GLuint program);

public:
  Mesh();
  Mesh(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI);
  void loadVertexData(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI);
  GLuint getVertexBuf();
  GLuint getIndexBuf();
  virtual void display(GLuint program, const glm::mat4& viewPerspective);
};


class TransformableMesh : public Mesh {
  glm::mat4 transformation;

public:
  TransformableMesh();
  void translate(const glm::vec3& translation_vec);
  void rotate(float angle, const glm::vec3& axis);
  void display(GLuint program, const glm::mat4& viewPerspective);
};

#endif
