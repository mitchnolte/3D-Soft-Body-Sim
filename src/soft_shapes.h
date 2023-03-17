#ifndef SOFT_SHAPES_H
#define SOFT_SHAPES_H

#include <utility>
#include "soft_body.h"


struct CellMass {
  Mass* mass;
  unsigned int index = -1;
};

/**
 * Stores positional data about masses for the construction of the mass-spring
 * structure of a cube. Used to connect new cells to previously created cells.
 * 
 * Cubes are constructed from cubic cells of 9 masses with 1 at each vertex and
 * 1 in the center. The vertex masses are connected to their adjacent vertices
 * with springs and all of them are connected to the central mass.
 * 
 * The masses are labeled with a low or high value (l or h) for the x, y, and z
 * coordinates, in that order. For example, lll is the mass at the low position
 * for each axis, or (-0.5, -0.5, -0.5) for a unit cube centered at the origin.
 */
struct CubeCell {
  CellMass center;
  CellMass lll;
  CellMass llh;
  CellMass lhl;
  CellMass lhh;
  CellMass hll;
  CellMass hlh;
  CellMass hhl;
  CellMass hhh;
};


class SoftCube : public SoftBody {
  int numCells;

  void buildStructure(Vector position, float size);
  CubeCell buildCell(Vector& cellCenter, float cellSize,
                     CubeCell& cellX, CubeCell& cellY, CubeCell& cellZ);

public:
  SoftCube(Vector position=Vector(3), float size=1, int numCells=27);
};

#endif
