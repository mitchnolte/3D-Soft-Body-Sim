#ifndef SOFT_CUBE_H
#define SOFT_CUBE_H

#include <GL/glew.h>
#include <vector>
#include "soft_body.h"
#include "mesh.h"


/**
 * @brief Stores positional data about masses for the construction of the
 *        mass-spring structure of a cube. Used to connect new cells to
 *        previously created cells by locating the desired masses in the list
 *        using the indices stored in the CubeCell structs.
 *
 *        Cubes are constructed from cubic cells of 9 masses with 1 at each
 *        vertex and 1 in the center. The vertex masses are connected to their
 *        adjacent vertices with springs and all of them are connected to the
 *        central mass.
 *
 *        The masses are labeled with a low or high value (l or h) for the x, y,
 *        and z coordinates, in that order. For example, lll is the mass at the
 *        low position for each axis, or (-0.5, -0.5, -0.5) for a unit cube
 *        centered at the origin.
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

// 3D grid of CubeCells for the construction of a cube's mass-spring structure
typedef std::vector<std::vector<std::vector<CubeCell>>> CubeCellGrid;


/**
 * @brief SoftBody implementation that arranges the point masses into a cube
 *        built from 2x2x2 cells of point masses with springs connecting
 *        adjacent masses, and an additional mass in the center connected to
 *        each other mass.
 */
class SoftCube : public SoftBody {
  int cornerMasses[8];  // Indices of corner masses
  int centerMass;       // Index of central mass


  CubeCell buildCell(const Vector& cellCenter, CubeCell& cellX, CubeCell& cellY,
                     CubeCell& cellZ, double cellSize, double k, double c);

  SoftBodyMesh buildMesh(const CubeCellGrid& cells, int cellsPerAxis, const Material& material);
  void getMeshIndices(std::vector<GLuint>& indices, std::vector<GLuint>& massIndices,
                      std::vector<GLfloat>& vertices, const CubeCellGrid& cells, int cellsPerAxis);

  void duplicateVertex(std::vector<GLfloat>& vertices, std::vector<GLuint>& indices,
                       std::vector<GLuint>& massIndices, GLuint i);

public:
  SoftCube();
  SoftCube(const SoftCube& cube);
  SoftBodyMesh buildStructure(const Vector& position=Vector(3), double size=1, int cellsPerAxis=3,
                              double k=10, double c=0.2, const Material& material={{1,0,0,1}, 1});
  Vector getCenterOfMass() const;
};

#endif
