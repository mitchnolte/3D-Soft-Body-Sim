#ifndef COLLISION_DATA_H
#define COLLISION_DATA_H

/**
 * Specifies a type of collision, for example a collision between a soft body
 * and a rigid rectangular prism. The two object types in the collision are
 * separated by an "_" in the enum constant name.
 */
enum class CollType {
  soft_rigidRectPrism
};

/**
 * Stores information about a collision.
 */
struct Collision {
  CollType type;  // Type of collision
  double   time;  // Time of collision
};

/**
 * Soft-rigid body collision; stores information about a collision between a
 * single point mass of a soft body and rigid body.
 */
struct S_R_Coll : Collision {
  int softBody;   // Index of soft body in simulation's soft body list
  int rigidBody;  // Index of rigid body in simulation's rigid body list
  int mass;       // Index of the point mass in the soft body's mass list
};

/**
 * Soft-body / rigid-rectangular-prism collision; stores information about a
 * collision between a single point mass of a soft body and a face of a rigid
 * rectangular prism.
 */
struct S_RRP_Coll : S_R_Coll {
  int face;       // Index of the face in the rigid body's face list
};

#endif
