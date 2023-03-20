#ifndef MESH_H
#define MESH_H

#include <GL/glew.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include "vector.h"


/**
 * Uniform block for a mesh's material properties.
 */
struct Material {
  GLfloat colour[4];
  GLfloat reflectivity;
  GLfloat padding[3] = {0, 0, 0};
};


class Mesh {
protected:
  GLuint vertexBuf;
  GLuint indexBuf;
  GLuint materialBuf;
  GLuint numTris;
  GLuint64 vertexSize;  // Size of vertices in bytes

  void sendVertexData(GLuint program);

public:
  static std::vector<GLfloat> computeNormals(GLfloat* vertices, GLuint* indices, int numV, int numI);
  static void triangleNormal(GLfloat normal[], GLfloat p1[3], GLfloat p2[3], GLfloat p3[3]);

  Mesh();
  Mesh(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI,
       const Material& material);
  void setMaterial(const Material& material);
  void loadVertexData(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI);
  void updateVertexData(GLfloat vertices[]);
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
