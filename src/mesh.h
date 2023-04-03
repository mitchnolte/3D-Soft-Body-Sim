#ifndef MESH_H
#define MESH_H

#include <GL/glew.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include "vector.h"
class SoftBody;


/**
 * @brief Uniform block for a mesh's material properties.
 */
struct Material {
  GLfloat colour[4];
  GLfloat reflectivity;
  GLfloat padding[3] = {0, 0, 0};   // Padding so uniform block is multiple of 16 bytes
};


/**
 * @brief Manages the data and rendering of a polygonal mesh.
 */
class Mesh {
protected:
  GLuint vertexBuf;
  GLuint indexBuf;
  GLuint materialBuf;
  GLuint numTris;
  GLuint64 vertexSize;  // Size of vertices in bytes

  void sendVertexData(GLuint program);

public:
  static void computeNormals(GLfloat normals[], GLfloat vertices[], GLuint indices[],
                             int numV, int numI);
  static void triangleNormal(GLfloat normal[], GLfloat p1[3], GLfloat p2[3], GLfloat p3[3]);

  Mesh();
  Mesh(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI,
       const Material& material);
  void setMaterial(const Material& material);
  void loadVertexData(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI);
  virtual void display(GLuint program, const glm::mat4& viewPerspective);
};


/**
 * @brief Manages the data and rendering of a polygonal mesh that represents a
 *        soft body.
 */
class SoftBodyMesh : public Mesh {
  const SoftBody* body;             // Soft body represented by the mesh
  std::vector<GLuint> massIndices;  // Index of mass in list from body for each mesh vertex
  std::vector<GLuint> indices;      // Triangle vertex indices

public:
  SoftBodyMesh();
  SoftBodyMesh(GLfloat vertices[], GLfloat normals[], GLuint indices[],
               const std::vector<GLuint>& massIndices, int numV, int numI, const Material& material);
  void bindBody(const SoftBody* body);
  void update();
  void display(GLuint program, const glm::mat4& viewProjection);
};

#endif
