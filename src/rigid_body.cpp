#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "rigid_body.h"
#include "soft_body.h"
#include "mesh.h"


const Vector& RigidBody::getCenterOfMass() const {
  return centerOfMass;
}

double RigidBody::getBoundingRadius() const {
  return boundingRadius;
}

/**
 * @brief Returns a pair with the static and kinetic friction coefficients of
 *        the rigid body's surface.
 */
const std::pair<double, double>& RigidBody::getFrictionCoefficients() const {
  return friction;
}


/*******************************************************************************
 *  RIGID RECTANGULAR PRISM
 ******************************************************************************/

RigidRectPrism::RigidRectPrism() {}

RigidRectPrism::RigidRectPrism(const RigidRectPrism& rect) {
  this->centerOfMass   = rect.centerOfMass;
  this->boundingRadius = rect.boundingRadius;
  this->friction       = rect.friction;
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
 * @brief  Rigid rectangular prism constructor.
 *
 * @param  centerOfMass     Position of the center of mass.
 * @param  xLen             X-axis side length.
 * @param  yLen             Y-axis side length. Defaults to x-axis length.
 * @param  zLen             Z-axis side length. Defaults to x-axis length.
 * @param  rotateAngle      Rotation angle for orientation in radians.
 * @param  rotateAxis       Rotation axis for orientation.
 * @param  staticFriction   Static friction coefficient of surface.
 * @param  kineticFriction  Kinetic friction coefficient of surface.
 */
RigidRectPrism::RigidRectPrism(const Vector& centerOfMass, float xLen, float yLen, float zLen,
                               float rotateAngle, const Vector& rotateAxis,
                               double staticFriction, double kineticFriction)
{
  this->centerOfMass = centerOfMass;
  this->friction     = std::make_pair(staticFriction, kineticFriction);

  xLen /= 2;
  if(yLen == 0) yLen = xLen;
  else          yLen /= 2;
  if(zLen == 0) zLen = xLen;
  else          zLen /= 2;
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

  // Define faces (side comments are relative to viewing the body when facing
  //               towards +y with +z = up direction and no rotation of the body)
  faces[0] = {{0, 1, 2, 3}, planeNormal(vertices[0], vertices[1], vertices[2])};  // Bottom
  faces[1] = {{4, 5, 6, 7}, planeNormal(vertices[4], vertices[5], vertices[6])};  // Top
  faces[2] = {{0, 3, 5, 4}, planeNormal(vertices[0], vertices[3], vertices[5])};  // Front
  faces[3] = {{1, 7, 6, 2}, planeNormal(vertices[1], vertices[7], vertices[6])};  // Back
  faces[4] = {{0, 4, 7, 1}, planeNormal(vertices[1], vertices[0], vertices[4])};  // Left
  faces[5] = {{3, 2, 6, 5}, planeNormal(vertices[3], vertices[2], vertices[6])};  // Right
}


const Vector* RigidRectPrism::getVertices() const {
  return vertices;
}

const Quad* RigidRectPrism::getFaces() const {
  return faces;
}


/**
 * @brief Builds a display mesh to represent the prism.
 */
Mesh RigidRectPrism::buildMesh() {
  int numV = 72;    // 6 faces * 4 vertices per face * 3 coordinates per vertex
  int numI = 36;    // 6 faces * 2 triangles per face * 3 indices per triangle
  GLfloat meshVertices[numV];
  GLfloat meshNormals[numV];
  GLuint  meshIndices[numI];
  Material material = {{0.5, 0.5, 0.5, 1.0}, 0.2};

  for(int i=0; i<6; i++) {
    int* faceIndices = faces[i].vertices;

    // Vertices and normals
    for(int j=0; j<4; j++) {
      int k = 12*i + 3*j;
      meshVertices[k]   = vertices[faceIndices[j]][0];
      meshVertices[k+1] = vertices[faceIndices[j]][1];
      meshVertices[k+2] = vertices[faceIndices[j]][2];

      meshNormals[k]   = faces[i].normal[0];
      meshNormals[k+1] = faces[i].normal[1];
      meshNormals[k+2] = faces[i].normal[2];
    }

    // Indices
    int l = 6*i;
    int m = 4*i;
    meshIndices[l]   = m;
    meshIndices[l+1] = m+1;
    meshIndices[l+2] = m+2;
    meshIndices[l+3] = m+2;
    meshIndices[l+4] = m+3;
    meshIndices[l+5] = m;
  }

  return Mesh(meshVertices, meshNormals, meshIndices, numV, numI, material);
}


/**
 * @brief  Calculates the signed distance from a point to each face of the
 *         prism. If colliding is true, the function will end the first time the
 *         point is determined to be on the outer side of a face and will return
 *         a std::vector with only -1 in it. If colliding is false, the function
 *         will instead return the indices of the faces the point is on the
 *         outer side of.
 *
 * @param  distances  Destination array. The value at a given index is set to
 *                    the distance from the face at the same index of the face
 *                    array.
 * @param  point      The point.
 * @param  colliding  Whether the point must be colliding with the prism or not.
 *
 * @return std::vector containing indices of faces that the point is on the
 *         outer side of or -1 if the point must be colliding but isn't.
 */
std::vector<int> RigidRectPrism::distanceFromFaces(double distances[6], const Vector& point, 
                                                   bool colliding)
{
  std::vector<int> noCollisionFaces;

  // Direction from 2 opposing vertices of the prism to the mass position
  Vector direction[2] = {point - vertices[0], point - vertices[6]};

  for(int i=0; i<6; i++) {
    distances[i] = vecDot(direction[i&1], faces[i].normal);   // Bitwise & to determine if i is even
    if(distances[i] > 0) {
      // printf("%f\n", distances[i]);
      if(colliding) return std::vector<int>(1, -1);
      noCollisionFaces.push_back(i);

      if(i&1 == 0) i++;   // Skip checking opposing face since the point must be inside
    }
  }

  if(colliding)
    return std::vector<int>(1);
  return noCollisionFaces;
}


/**
 * @brief  Calculates the point where a line segment intersects with a plane.
 *
 * @param  posA   Starting position of the line segment.
 * @param  posB   Ending position of the line segment.
 * @param  distA  Signed distance from posA to the plane.
 * @param  distB  Signed distance from posB to the plane.
 * 
 * @return Intersection point.
 */
Vector RigidRectPrism::planeIntersection(const Vector& posA, const Vector& posB,
                                         double distA, double distB)
{
  return (distA*posB - distB*posA) / (distA - distB);
}


/**
 * @brief  Checks every surface mass in the given soft body for collision with
 *         the prism.
 *
 * @param  softBody  Pointer to the soft body.
 * @param  state     State of the soft body at the end of the update step.
 * @param  tStart    Starting time of update step.
 * @param  dt        Duration of update step.
 *
 * @return List of collision data structs in an arbitrary order. Soft and rigid
 *         body indices are not initialized.
 */
std::vector<Collision*> RigidRectPrism::detectCollisions(const SoftBody* softBody,
                                                     const VecList& state, double tStart, double dt)
{
  std::vector<Collision*> collisions;
  const std::vector<Mass>& masses      = softBody->getMasses();
  for(int i=0; i<masses.size(); i++) {

    // Check if mass is colliding with prism
    const Vector& posEnd = state[i][Mass::POS];
    double distEnd[6];
    if(distanceFromFaces(distEnd, posEnd, true)[0] == -1)
      continue;

    // Determine which face the mass collided with
    Vector posStart = masses[i].getPos();
    double distStart[6];
    std::vector<int> possibleFaces = distanceFromFaces(distStart, posStart, false);
    if(possibleFaces.size() == 0)
      continue;

    // Find linearly approximated point of collision with face plane
    int face = possibleFaces[0];
    Vector approxCollPoint = planeIntersection(posStart, posEnd, distStart[face], distEnd[face]);

    // If multiple possible points, find a point that is within a face
    // (with 2D equivalent of RigidRectPrism::distanceFromFaces with mandatory collision)
    if(possibleFaces.size() > 1) {
      for(int i=1; i<possibleFaces.size(); i++) {
        Vector direction = approxCollPoint - vertices[faces[face].vertices[0]];

        // Indices of faces that contain the normals of the relevant face's edges
        int facesForNormals[2] = {face<2? 2:0, face<4? 4:2};

        if((vecDot(direction, faces[facesForNormals[0]].normal) > 0) &&
           (vecDot(direction, faces[facesForNormals[1]].normal) > 0))
        {
          direction = approxCollPoint - vertices[faces[face].vertices[2]];
          if((vecDot(direction, faces[facesForNormals[0] + 1].normal) > 0) &&
             (vecDot(direction, faces[facesForNormals[1] + 1].normal) > 0))
          {
            continue;
          }
        }

        face = possibleFaces[i];
        approxCollPoint = planeIntersection(posStart, posEnd, distStart[face], distEnd[face]);
      }
    }

    // Find time of linearly approximated collision
    double tColl = tStart;
    for(int i=0; i<3; i++) {
      if(posStart[i] != posEnd[i]) {
        tColl += dt * (approxCollPoint[i] - posStart[i]) / (posEnd[i] - posStart[i]);
        break;
      }
    }

    // Add collision to list
    S_RRP_Coll coll;
    coll.type = CollType::soft_rigidRectPrism;
    coll.time = tColl;
    coll.e    = 0.5;
    coll.mass = i;
    coll.face = face;
    collisions.push_back(new S_RRP_Coll(coll));
  }

  return collisions;
}
