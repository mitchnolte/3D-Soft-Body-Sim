#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include "soft_body.h"
#include "mesh.h"
#include "collision_data.h"


class RigidBody {
protected:
  Vector centerOfMass;    // Position of the center of mass
  double boundingRadius;  // Radius of bounding sphere for collision detection

public:
  const Vector& getCenterOfMass() const;
  double getBoundingRadius() const;

  virtual Mesh buildMesh() = 0;
  virtual std::vector<Collision*> detectCollisions(const SoftBody* softBody, const VecList& state,
                                                   double tStart, double dt) = 0;
};


struct Quad;

/**
 * Immobile rigid body rectangular prism.
 */
class RigidRectPrism : public RigidBody {
  Vector vertices[8];   // Positions of the prism's vertices
  Quad faces[6];        // Faces of the prism for collision detection

  std::vector<int> distanceFromFaces(double distances[6], const Vector& point, bool colliding);
  Vector planeIntersection(const Vector& posA, const Vector& posB, double distA, double distB);

public:
  RigidRectPrism();
  RigidRectPrism(const RigidRectPrism& rect);
  RigidRectPrism(const Vector& centerOfMass, float xLen, float zLen=0, float yLen=0,
                 float rotateAngle=0.0, const Vector& rotateAxis=Vector{0,0,1});
  Mesh buildMesh();
  std::vector<Collision*> detectCollisions(const SoftBody* softBody, const VecList& state,
                                           double tStart, double dt);
};


/**
 * Stores information about a quadrilateral face of a rigid body, which should
 * be planar. Vertex indices are expected to be given with counterclockwise
 * winding order.
 */
struct Quad {
  int vertices[4];  // Indices of vertices in body's vertex list
  Vector normal;    // Normal vector of face
};

#endif
