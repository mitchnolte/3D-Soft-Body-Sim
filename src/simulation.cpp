#include "simulation.h"
#include "collision_data.h"


/**
 * @brief Convenience function to append a std::vector of collision data to the
 *        main collision data std::vector in Simulation::update.
 * @param colls Main collision list.
 * @param newColls New collision data to append.
 */
void addCollisions(std::vector<Collision*>& colls, const std::vector<Collision*>& newColls) {
  colls.insert(colls.end(), newColls.begin(), newColls.end());
}


/**
 * @brief Constructor.
 * @param dt Time between each update.
 * @param iterationsPerUpdate Number of RK4 iterations per update.
 */
Simulation::Simulation(double dt, int iterationsPerUpdate) {
  this->time = 0;
  this->dt = dt;
  this->iterationsPerUpdate = iterationsPerUpdate;
}

Simulation::~Simulation() {
  for(int i=0; i<softBodies.size(); i++) {
    delete softBodies[i];
  }
  for(int i=0; i<rigidBodies.size(); i++) {
    delete rigidBodies[i];
  }
}

void Simulation::addBody(const SoftCube& body) {
  softBodies.push_back(new SoftCube(body));
}

void Simulation::addBody(const RigidRectPrism& body) {
  rigidBodies.push_back(new RigidRectPrism(body));
}

const std::vector<SoftBody*>& Simulation::getSoftBodies() const {
  return softBodies;
}

void Simulation::update() {
  double tEnd = time + dt;

  // Calculate soft body states at end of update step
  std::vector<const VecList*> softBodyStates(softBodies.size());
  for(int i=0; i<softBodies.size(); i++) {
    softBodyStates[i] = &softBodies[i]->calculateUpdatedState(tEnd, iterationsPerUpdate);
  }

  // Collision detection
  std::vector<Collision*> collisions;
  for(int i=0; i<softBodyStates.size(); i++) {
    for(RigidBody* rigidBody : rigidBodies) {

      // Bounding sphere test
      if(vecNorm(rigidBody->getCenterOfMass() - softBodies[i]->getCenterOfMass())
         <= rigidBody->getBoundingRadius() + softBodies[i]->getBoundingRadius())
      {
        // Full collision test
        addCollisions(collisions, rigidBody->detectCollisions(softBodies[i], *softBodyStates[i],
                                                              time, dt));
      }
    }
  }

  // Collision response

  // Update soft bodies
  for(int i=0; i<softBodies.size(); i++) {
    softBodies[i]->update(*softBodyStates[i]);
  }
  time = tEnd;
}
