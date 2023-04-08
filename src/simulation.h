#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
class SoftBody;
class SoftCube;
class RigidBody;
class RigidRectPrism;


/**
 * @brief Manages and updates the objects in the simulation.
 */
class Simulation {
  std::vector<SoftBody*> softBodies;
  std::vector<RigidBody*> rigidBodies;
  double dt;                           // Time step duration
  double time;
  int iterationsPerUpdate;             // Number of RK4 iterations per time step

public:
  Simulation(double dt, int iterationsPerUpdate=1);
  ~Simulation();
  void addBody(const SoftCube& body);
  void addBody(const RigidRectPrism& body);
  const std::vector<SoftBody*>& getSoftBodies() const;
  void update();
};

#endif
