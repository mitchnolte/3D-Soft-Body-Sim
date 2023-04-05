#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include "vector.h"
#include "collision_data.h"
class SoftBody;
class Mesh;


/**
 * @brief Interface for an immobile rigid body. Handles collision detection with
 *        soft bodies.
 */
class RigidBody {
protected:
  Vector centerOfMass;                // Position of the center of mass
  double boundingRadius;              // Radius of bounding sphere for collision detection
  std::pair<double, double> friction; // Static and kinetic friction coefficients of surface

public:
  const Vector& getCenterOfMass() const;
  double getBoundingRadius() const;
  const std::pair<double, double>& getFrictionCoefficients() const;

  virtual Mesh buildMesh() = 0;
  virtual std::vector<Collision*> detectCollisions(const SoftBody* softBody, const VecList& state,
                                                   double tStart, double dt) = 0;
};


/**
 * @brief Stores information about a quadrilateral face of a rigid body, which
 *        should be planar. Vertex indices are expected to be given with
 *        counterclockwise winding order.
 */
struct Quad {
  int vertices[4];  // Indices of vertices in body's vertex list
  Vector normal;    // Normal vector of face
};


/**
 * @brief Immobile rigid body rectangular prism.
 */
class RigidRectPrism : public RigidBody {
  Vector vertices[8];     // Positions of the prism's vertices
  Quad faces[6];          // Faces of the prism for collision detection

  std::vector<int> distanceFromFaces(double distances[6], const Vector& point, bool colliding);
  Vector planeIntersection(const Vector& posA, const Vector& posB, double distA, double distB);

public:
  RigidRectPrism();
  RigidRectPrism(const RigidRectPrism& rect);
  RigidRectPrism(const Vector& centerOfMass, float xLen, float yLen=0, float zLen=0,
                 float rotateAngle=0.0, const Vector& rotateAxis=Vector{0,0,1},
                 double staticFriction=0.5, double kineticFriction=0.3);
  const Vector* getVertices() const;
  const Quad* getFaces() const;
  Mesh buildMesh();
  std::vector<Collision*> detectCollisions(const SoftBody* softBody, const VecList& state,
                                           double tStart, double dt);
};

#endif
