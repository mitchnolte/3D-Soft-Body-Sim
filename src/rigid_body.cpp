#include "rigid_body.h"
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


const Vector& RigidBody::getCenterOfMass() const {
  return centerOfMass;
}

double RigidBody::getBoundingRadius() const {
  return boundingRadius;
}


/*******************************************************************************
 *  RIGID RECTANGULAR PRISM
 ******************************************************************************/

RigidRectPrism::RigidRectPrism() {}

RigidRectPrism::RigidRectPrism(const RigidRectPrism& rect) {
  this->centerOfMass = rect.centerOfMass;
  this->boundingRadius = rect.boundingRadius;
  for(int i=0; i<8; i++)
    this->vertices[i] = rect.vertices[i];

  for(int i=0; i<6; i++) {
    this->faces[i] = rect.faces[i];
    for(int j=0; j<4; j++) {
      this->faces[i].vertices[j] = rect.faces[i].vertices[j];
    }
  }
}


/**
 * @brief Constructor.
 * @param centerOfMass Position of the center of mass (in world coordinates).
 * @param xLen X-axis side length.
 * @param yLen Y-axis side length.
 * @param zLen Z-axis side length.
 * @param rotateAngle Rotation angle for orientation in radians.
 * @param rotateAxis Rotation axis for orientation.
 */
RigidRectPrism::RigidRectPrism(const Vector& centerOfMass, float xLen, float yLen, float zLen,
                               float rotateAngle, const Vector& rotateAxis)
{
  this->centerOfMass = centerOfMass;

  xLen /= 2;
  yLen /= 2;
  zLen /= 2;
  boundingRadius = vecNorm(Vector{xLen, yLen, zLen});

  // Create vertices centered on origin
  vertices[0] = Vector{-xLen, -yLen, -zLen};
  vertices[1] = Vector{-xLen,  yLen, -zLen};
  vertices[2] = Vector{ xLen,  yLen, -zLen};
  vertices[3] = Vector{ xLen, -yLen, -zLen};
  vertices[4] = Vector{-xLen, -yLen,  zLen};
  vertices[5] = Vector{ xLen, -yLen,  zLen};
  vertices[6] = Vector{ xLen,  yLen,  zLen};
  vertices[7] = Vector{-xLen,  yLen,  zLen};

  // Rotate body if necessary and translate to given position
  if(rotateAngle != 0.0) {
    Vector rAxis = normalize(rotateAxis);
    glm::vec3 axis(rAxis[0], rAxis[1], rAxis[2]);
    glm::mat4 rotation = glm::rotate(glm::mat4(1), rotateAngle, axis);
    for(int i=0; i<8; i++)
      vertices[i] = matVecMul(rotation, vertices[i]) + centerOfMass;
  }
  else {
    for(int i=0; i<8; i++)
      vertices[i] += centerOfMass;
  }

  // Define faces
  faces[0] = {{0, 1, 2, 3}, planeNormal(vertices[0], vertices[1], vertices[2])};
  faces[1] = {{4, 5, 6, 7}, planeNormal(vertices[4], vertices[5], vertices[6])};
  faces[2] = {{0, 3, 5, 4}, planeNormal(vertices[0], vertices[3], vertices[5])};
  faces[3] = {{1, 7, 6, 2}, planeNormal(vertices[1], vertices[7], vertices[6])};
  faces[4] = {{1, 0, 4, 7}, planeNormal(vertices[1], vertices[0], vertices[4])};
  faces[5] = {{3, 2, 6, 5}, planeNormal(vertices[3], vertices[2], vertices[6])};
}


Mesh RigidRectPrism::buildMesh() {
  int numV = 72;  // 6 faces * 4 vertices per face * 3 coordinates per vertex
  int numI = 36;  // 6 faces * 2 triangles per face * 3 indices per triangle
  GLfloat meshVertices[numV];
  GLfloat meshNormals[numV];
  GLuint  meshIndices[numI];
  Material material = {{0.5, 0.5, 0.5, 1.0}, 0.2};

  for(int i=0; i<6; i++) {
    int* faceIndices = faces[i].vertices;
    int j = 12*i;

    // Vertices and normals
    for(int k=0; k<4; k++) {
      int l = 3*k;
      meshVertices[j+l]   = vertices[faceIndices[k]][0];
      meshVertices[j+l+1] = vertices[faceIndices[k]][1];
      meshVertices[j+l+2] = vertices[faceIndices[k]][2];

      meshNormals[j+l]   = faces[i].normal[0];
      meshNormals[j+l+1] = faces[i].normal[1];
      meshNormals[j+l+2] = faces[i].normal[2];
    }

    // Indices
    int m = 6*i;
    int n = 4*i;
    meshIndices[m]   = n;
    meshIndices[m+1] = n+1;
    meshIndices[m+2] = n+2;
    meshIndices[m+3] = n+2;
    meshIndices[m+4] = n+3;
    meshIndices[m+5] = n;
  }

  return Mesh(meshVertices, meshNormals, meshIndices, numV, numI, material);
}
