#ifndef SOFT_BODY_FACTORY_H
#define SOFT_BODY_FACTORY_H

#include <utility>
#include "soft_body.h"
#include "mesh.h"


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
  unsigned int center;
  unsigned int lll;
  unsigned int llh;
  unsigned int lhl;
  unsigned int lhh;
  unsigned int hll;
  unsigned int hlh;
  unsigned int hhl;
  unsigned int hhh;
};


class SoftBodyFactory {
  CubeCell buildCubeCell(std::vector<Mass>& masses, Vector& cellCenter, float cellSize,
                         CubeCell& cellX, CubeCell& cellY, CubeCell& cellZ);
  Mesh buildCubeMesh(const std::vector<int>& surfaceMasses, const SoftBody& cube,
                     std::vector<std::vector<std::vector<CubeCell>>>& cells, int numCells);
  std::vector<GLuint> getCubeVertexIndices(const std::vector<int>& surfaceMasses, int cellsPerAxis,
                                           std::vector<std::vector<std::vector<CubeCell>>>& cells);

public:
  std::pair<SoftBody, Mesh> buildCube(Vector position=Vector(3), float size=1,
                                      unsigned int numCells=27, float k=10, float c=0.2);
};

#endif
