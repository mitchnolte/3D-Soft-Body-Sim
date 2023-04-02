#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include "soft_body.h"
#include "mesh.h"


class RigidBody {
protected:
  Vector centerOfMass;    // Position of the center of mass
  double boundingRadius;  // Radius of bounding sphere for collision detection

public:
  const Vector& getCenterOfMass() const;
  double getBoundingRadius() const;

  virtual Mesh buildMesh() = 0;
  // virtual void detectCollision(const SoftBody* softBody) = 0;
};


/**
 * Stores information about a four sided face of a rigid body, which should be
 * planar. Vertex indices are expected to have counterclockwise winding order.
 */
struct QuadFace {
  int vertices[4];  // Indices of vertices in body's vertex list
  Vector normal;    // Normal vector of face
};

class RigidRectPrism : public RigidBody {
  Vector vertices[8];   // Positions of the prism's vertices
  QuadFace faces[6];    // Faces of the prism for collision detection

public:
  RigidRectPrism();
  RigidRectPrism(const RigidRectPrism& rect);
  RigidRectPrism(const Vector& centerOfMass, float xLen, float yLen, float zLen,
                 float rotateAngle=0.0, const Vector& rotateAxis=Vector{0,0,1});
  Mesh buildMesh();
};

#endif
