#include "soft_cube.h"
#include "soft_body.h"


/**
 * @brief  Soft body cube constructor.
 * 
 * @param  mass      Mass of the cube.
 * @param  friction  Extra friction coefficient used to stabilize structure.
 */
SoftCube::SoftCube(double mass, double friction) {
  this->mass = mass;
  this->friction = friction;
}

SoftCube::SoftCube(const SoftCube& cube) : SoftBody(cube) {
  for(int i=0; i<8; i++)
    cornerMasses[i] = cube.cornerMasses[i];
}

/**
 * @brief Approximates the cube's center of mass by calculating the average
 *        position of the 8 masses located on one of the cube's corners.
 */
Vector SoftCube::getCenterOfMass() {
  return (masses[cornerMasses[0]].getPos() + masses[cornerMasses[1]].getPos() +
          masses[cornerMasses[2]].getPos() + masses[cornerMasses[3]].getPos() +
          masses[cornerMasses[4]].getPos() + masses[cornerMasses[5]].getPos() +
          masses[cornerMasses[6]].getPos() + masses[cornerMasses[7]].getPos()) / 8;
}


/*******************************************************************************
 *  CONSTRUCTION
 ******************************************************************************/

GLuint indexOf(const std::vector<int>& v, GLuint i) {
  return std::distance(v.begin(), std::find(v.begin(), v.end(), i));
}

void duplicateVertex(std::vector<GLfloat>& vertices, std::vector<GLuint>& indices,
                     std::vector<GLuint>& massIndices, GLuint i)
{
  vertices.push_back(vertices[3*indices[i]]);
  vertices.push_back(vertices[3*indices[i] + 1]);
  vertices.push_back(vertices[3*indices[i] + 2]);
  massIndices.push_back(indices[i]);
  indices[i] = vertices.size()/3 - 1;
}


/**
 * @brief  Fills the index list with mesh vertex indices. Also adds new vertices
 *         to the given vertex list by duplicating the vertices on the edges of
 *         the cube so that each face of the cube has a separate copy of the
 *         vertex with its own normal vector.
 *
 * @param  indices       Destination vector for mesh vertex indices.
 * @param  massIndices   Destination vector for indices of the masses that
 *                       correspond to each mesh vertex.
 * @param  vertices      Vertex list.
 * @param  cells         Cell grid.
 * @param  cellsPerAxis  Number of cells per axis in the cube.
 */
void SoftCube::getMeshIndices(std::vector<GLuint>& indices, std::vector<GLuint>& massIndices,
                              std::vector<GLfloat>& vertices, const CubeCellGrid& cells,
                              int cellsPerAxis)
{
  const CubeCell* cell;

  // Triangles on x-axis min. and max. sides (left and right)
  for(int y=1; y<=cellsPerAxis; y++) {
    for(int z=1; z<=cellsPerAxis; z++) {
      cell = &cells[0][y][z];
      indices.insert(indices.end(), {cell->hhl, cell->hll, cell->hhh,
                                     cell->hhh, cell->hll, cell->hlh});

      cell = &cells[cellsPerAxis][y][z];
      indices.push_back(indexOf(surfaceMassIndices, cell->hhh));
      indices.push_back(indexOf(surfaceMassIndices, cell->hll));
      indices.push_back(indexOf(surfaceMassIndices, cell->hhl));
      indices.push_back(indexOf(surfaceMassIndices, cell->hlh));
      indices.push_back(indexOf(surfaceMassIndices, cell->hll));
      indices.push_back(indexOf(surfaceMassIndices, cell->hhh));

      // indices.insert(indices.end(), {indexOf(surfaceMassIndices, cell->hhh),
      //                                indexOf(surfaceMassIndices, cell->hll),
      //                                indexOf(surfaceMassIndices, cell->hhl),
      //                                indexOf(surfaceMassIndices, cell->hlh),
      //                                indexOf(surfaceMassIndices, cell->hll),
      //                                indexOf(surfaceMassIndices, cell->hhh)});
    }
  }

  // Triangles on y-axis min. and max. sides (front and back)
  for(int x=1; x<=cellsPerAxis; x++) {
    for(int z=1; z<=cellsPerAxis; z++) {
      cell = &cells[x][0][z];
      indices.insert(indices.end(), {cell->lhl, cell->hhl, cell->lhh,
                                     cell->lhh, cell->hhl, cell->hhh});

      cell = &cells[x][cellsPerAxis][z];
      indices.push_back(indexOf(surfaceMassIndices, cell->lhh));
      indices.push_back(indexOf(surfaceMassIndices, cell->hhl));
      indices.push_back(indexOf(surfaceMassIndices, cell->lhl));
      indices.push_back(indexOf(surfaceMassIndices, cell->hhh));
      indices.push_back(indexOf(surfaceMassIndices, cell->hhl));
      indices.push_back(indexOf(surfaceMassIndices, cell->lhh));

      // Duplicate edge vertices so they have separate normals on each side for lighting
      int i = indices.size() - 1;
      if(x == 1) {
        if(z == 1) {
          duplicateVertex(vertices, indices, massIndices, i-11);
          duplicateVertex(vertices, indices, massIndices, i-3);
        }else {
          indices[i-11] = indices[i-21];
          indices[i-3]  = indices[i-17];
        }
        duplicateVertex(vertices, indices, massIndices, i-9);
        duplicateVertex(vertices, indices, massIndices, i);
        indices[i-8] = indices[i-9];
        indices[i-5] = indices[i];
      }
      if(x == cellsPerAxis) {
        if(z == 1) {
          duplicateVertex(vertices, indices, massIndices, i-10);
          duplicateVertex(vertices, indices, massIndices, i-1);
          indices[i-7]  = indices[i-10];
          indices[i-4]  = indices[i-1];
        }else {
          indices[i-10] = indices[i-18];
          indices[i-7]  = indices[i-18];
          indices[i-1]  = indices[i-14];
          indices[i-4]  = indices[i-14];
        }
        duplicateVertex(vertices, indices, massIndices, i-6);
        duplicateVertex(vertices, indices, massIndices, i-2);
      }
    }
  }

  // Triangles on z-axis min. and max. sides (bottom and top)
  for(int x=1; x<=cellsPerAxis; x++) {
    for(int y=1; y<=cellsPerAxis; y++) {
      cell = &cells[x][y][0];
      indices.insert(indices.end(), {cell->llh, cell->lhh, cell->hlh,
                                     cell->hlh, cell->lhh, cell->hhh});

      cell = &cells[x][y][cellsPerAxis];
      indices.push_back(indexOf(surfaceMassIndices, cell->hlh));
      indices.push_back(indexOf(surfaceMassIndices, cell->lhh));
      indices.push_back(indexOf(surfaceMassIndices, cell->llh));
      indices.push_back(indexOf(surfaceMassIndices, cell->hhh));
      indices.push_back(indexOf(surfaceMassIndices, cell->lhh));
      indices.push_back(indexOf(surfaceMassIndices, cell->hlh));

      // Duplicate edge vertices so they have separate normals on each side for lighting
      int i = indices.size() - 1;
      if(x == 1) {
        if(y == 1) {
          duplicateVertex(vertices, indices, massIndices, i-11);
          duplicateVertex(vertices, indices, massIndices, i-3);
        }else {
          indices[i-11] = indices[i-22];
          indices[i-3]  = indices[i-13];
        }
        duplicateVertex(vertices, indices, massIndices, i-10);
        duplicateVertex(vertices, indices, massIndices, i-1);
        indices[i-7]  = indices[i-10];
        indices[i-4]  = indices[i-1];
      }
      if(x == cellsPerAxis) {
        if(y == 1) {
          duplicateVertex(vertices, indices, massIndices, i-9);
          duplicateVertex(vertices, indices, massIndices, i);
          indices[i-8] = indices[i-9];
          indices[i-5] = indices[i];
        }else {
          indices[i-9] = indices[i-18];
          indices[i-8] = indices[i-18];
          indices[i]   = indices[i-14];
          indices[i-5] = indices[i-14];
        }
        duplicateVertex(vertices, indices, massIndices, i-6);
        duplicateVertex(vertices, indices, massIndices, i-2);
      }

      if(y == 1) {
        if(x < cellsPerAxis) {
          duplicateVertex(vertices, indices, massIndices, i-9);
          duplicateVertex(vertices, indices, massIndices, i);
          indices[i-8] = indices[i-9];
          indices[i-5] = indices[i];
        }if(x > 1) {
          indices[i-11] = indices[i - 8 - 12*cellsPerAxis];
          indices[i-3]  = indices[i - 12*cellsPerAxis];
        }
      }
      if(y == cellsPerAxis) {
        if(x < cellsPerAxis) {
          duplicateVertex(vertices, indices, massIndices, i-6);
          duplicateVertex(vertices, indices, massIndices, i-2);
        }if(x > 1) {
          indices[i-10] = indices[i - 6 - 12*cellsPerAxis];
          indices[i-7]  = indices[i - 6 - 12*cellsPerAxis];
          indices[i-1]  = indices[i - 2 - 12*cellsPerAxis];
          indices[i-4]  = indices[i - 2 - 12*cellsPerAxis];
        }
      }
    }
  }

  indices.shrink_to_fit();
  massIndices.shrink_to_fit();
}


/**
 * @brief  Builds the polygonal display mesh to represent the cube.
 * 
 * @param  cells         Cell grid.
 * @param  cellsPerAxis  Number of cells per axis in the cube.
 * @param  material      Material properties of the mesh.
 */
SoftBodyMesh SoftCube::buildMesh(const CubeCellGrid& cells, int cellsPerAxis,
                                 const Material& material)
{
  // Get vertex positions
  std::vector<GLfloat> vertices;
  std::vector<GLuint> massIndices;

  for(int i=0; i<surfaceMasses.size(); i++) {
    const Vector& state = masses[i].getState();
    vertices.insert(vertices.end(), {(float)state[0], (float)state[1], (float)state[2]});
    massIndices.push_back(i);
  }

  // Get vertex and mass indices
  std::vector<GLuint> indices;
  getMeshIndices(indices, massIndices, vertices, cells, cellsPerAxis);

  // Get normal vectors
  GLfloat normals[vertices.size()] = {0};
  Mesh::computeNormals(normals, vertices.data(), indices.data(), vertices.size(), indices.size());

  return SoftBodyMesh(vertices.data(), normals, indices.data(), massIndices,
                      vertices.size(), indices.size(), material);
}


/**
 * @brief  Creates new masses where necessary and connects them to the previous
 *         cells in each axis with springs.
 * 
 * @param  cellCenter  Central location of cell.
 * @param  cellX       Previous cell in x-axis.
 * @param  cellY       Previous cell in y-axis.
 * @param  cellZ       Previous cell in z-axis.
 * @param  cellSize    Side length of cell.
 * @param  k           Spring coefficient.
 * @param  c           Damping coefficient.
 */
CubeCell SoftCube::buildCell(const Vector& cellCenter, CubeCell& cellX, CubeCell& cellY,
                             CubeCell& cellZ, double cellSize, double k, double c)
{
  CubeCell cell;

  // Create new masses
  masses.emplace_back(cellCenter);
  cell.center = masses.size() - 1;
  masses.emplace_back(cellCenter + cellSize/2);
  cell.hhh = masses.size() - 1;

  // Connect to previous cell in z-axis
  cell.lll = cellZ.llh;
  cell.hll = cellZ.hlh;
  cell.lhl = cellZ.lhh;
  cell.hhl = cellZ.hhh;

  // Connect to previous cell in y-axis
  cell.llh = cellY.lhh;
  cell.hlh = cellY.hhh;

  // Connect to previous cell in x-axis
  cell.lhh = cellX.hhh;

  // Create Springs
  double centerDist = vecNorm(Vector(cellSize/2, 3));
  springs.emplace_back(cell.lll, cell.center, k, c, centerDist);
  springs.emplace_back(cell.llh, cell.center, k, c, centerDist);
  springs.emplace_back(cell.lhl, cell.center, k, c, centerDist);
  springs.emplace_back(cell.lhh, cell.center, k, c, centerDist);
  springs.emplace_back(cell.hll, cell.center, k, c, centerDist);
  springs.emplace_back(cell.hlh, cell.center, k, c, centerDist);
  springs.emplace_back(cell.hhl, cell.center, k, c, centerDist);
  springs.emplace_back(cell.hhh, cell.center, k, c, centerDist);
  springs.emplace_back(cell.lhh, cell.hhh, k, c, cellSize);
  springs.emplace_back(cell.hhl, cell.hhh, k, c, cellSize);
  springs.emplace_back(cell.hlh, cell.hhh, k, c, cellSize);

  return cell;
}


/**
 * @brief  Builds the mass-spring structure and the polygonal display mesh.
 *         Cube is built from 2x2x2 cells of point masses with springs
 *         connecting adjacent masses, and an additional mass in the center
 *         connected to each other mass. Mesh has one vertex for each point mass
 *         on the surface of the cube.
 *
 * @param  position      Central position of the cube. Defaults to (0, 0, 0).
 * @param  sideLengths   Side length of the cube. Defaults to 1 m.
 * @param  cellsPerAxis  Number of cells per axis of the cube.
 * @param  k             Spring coefficient.
 * @param  c             Damping coefficient.
 * @param  material      Mesh material properties. Defaults to red with full
 *                       reflectivity.
 *
 * @return Display mesh.
 */
SoftBodyMesh SoftCube::buildStructure(const Vector& position, double sideLengths, int cellsPerAxis,
                                      double k, double c, const Material& material)
{
  double numCells = pow(cellsPerAxis, 3);
  double cellSize = sideLengths / cellsPerAxis;   // Cell side length
  double halfCell = cellSize/2;
  Vector cellCenter;                              // Center of current cell
  Vector firstCellCenter = position - sideLengths/2 + halfCell;
  
  CubeCellGrid cells;
  cells.resize(cellsPerAxis+1);
  for(int i=0; i<cellsPerAxis+1; i++) {
    cells[i].resize(cellsPerAxis+1);
    for(int j=0; j<cellsPerAxis+1; j++) {
      cells[i][j].resize(cellsPerAxis+1);
    }
  }

  /**
   * Create temporary cells with surface masses for edge cases
   */

  // Cells below z-axis minimum
  CubeCell* cell = &cells[1][1][0];
  Vector massPos = firstCellCenter - halfCell;
  masses.emplace_back(massPos);
  cell->llh = masses.size()-1;
  surfaceMassIndices.push_back(cell->llh);

  massPos[1] += cellSize;
  masses.emplace_back(massPos);
  cell->lhh = masses.size()-1;
  surfaceMassIndices.push_back(cell->lhh);

  massPos += Vector{cellSize, -cellSize, 0};
  masses.emplace_back(massPos);
  cell->hlh = masses.size()-1;
  surfaceMassIndices.push_back(cell->hlh);

  massPos[1] += cellSize;
  masses.emplace_back(massPos);
  cell->hhh = masses.size()-1;
  surfaceMassIndices.push_back(cell->hhh);

  springs.emplace_back(cell->llh, cell->lhh, k, c, cellSize);
  springs.emplace_back(cell->llh, cell->hlh, k, c, cellSize);
  springs.emplace_back(cell->lhh, cell->hhh, k, c, cellSize);
  springs.emplace_back(cell->hlh, cell->hhh, k, c, cellSize);

  cellCenter = firstCellCenter + Vector{0, cellSize, -cellSize};
  for(int y=2; y<cellsPerAxis+1; y++) {
    CubeCell* priorCell = cell;
    cell = &cells[1][y][0];
    cell->llh = priorCell->lhh;
    cell->hlh = priorCell->hhh;

    masses.emplace_back(cellCenter + Vector{-halfCell, halfCell, halfCell});
    cell->lhh = masses.size()-1;
    surfaceMassIndices.push_back(cell->lhh);

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMassIndices.push_back(cell->hhh);

    springs.emplace_back(cell->llh, cell->lhh, k, c, cellSize);
    springs.emplace_back(cell->lhh, cell->hhh, k, c, cellSize);
    springs.emplace_back(cell->hlh, cell->hhh, k, c, cellSize);

    cellCenter[1] += cellSize;
  }

  cellCenter = firstCellCenter + Vector{cellSize, 0, -cellSize};
  for(int x=2; x<cellsPerAxis+1; x++) {
    CubeCell* priorCellX = &cells[x-1][1][0];
    cell = &cells[x][1][0];
    cell->llh = priorCellX->hlh;
    cell->lhh = priorCellX->hhh;

    masses.emplace_back(cellCenter + Vector{halfCell, -halfCell, halfCell});
    cell->hlh = masses.size()-1;
    surfaceMassIndices.push_back(cell->hlh);

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMassIndices.push_back(cell->hhh);

    springs.emplace_back(cell->llh, cell->hlh, k, c, cellSize);
    springs.emplace_back(cell->lhh, cell->hhh, k, c, cellSize);
    springs.emplace_back(cell->hlh, cell->hhh, k, c, cellSize);

    cellCenter[1] += cellSize;
    for(int y=2; y<cellsPerAxis+1; y++) {
      priorCellX = &cells[x-1][y][0];
      CubeCell* priorCellY = cell;
      cell = &cells[x][y][0];
      cell->llh = priorCellX->hlh;
      cell->lhh = priorCellX->hhh;
      cell->hlh = priorCellY->hhh;

      masses.emplace_back(cellCenter + halfCell);
      cell->hhh = masses.size()-1;
      surfaceMassIndices.push_back(cell->hhh);

      springs.emplace_back(cell->lhh, cell->hhh, k, c, cellSize);
      springs.emplace_back(cell->hlh, cell->hhh, k, c, cellSize);

      cellCenter[1] += cellSize;
    }
    cellCenter[1] = firstCellCenter[1];
    cellCenter[0] += cellSize;
  }


  // Cells below y-axis minimum
  cell = &cells[1][0][1];
  cell->lhl = cells[1][1][0].llh;
  cell->hhl = cells[1][1][0].hlh;

  massPos = firstCellCenter + Vector{-halfCell, -halfCell, halfCell};
  masses.emplace_back(massPos);
  cell->lhh = masses.size()-1;
  surfaceMassIndices.push_back(cell->lhh);

  massPos[0] += cellSize;
  masses.emplace_back(massPos);
  cell->hhh = masses.size()-1;
  surfaceMassIndices.push_back(cell->hhh);

  springs.emplace_back(cell->lhl, cell->lhh, k, c, cellSize);
  springs.emplace_back(cell->lhh, cell->hhh, k, c, cellSize);
  springs.emplace_back(cell->hhl, cell->hhh, k, c, cellSize);

  cellCenter = firstCellCenter + Vector{0, -cellSize, cellSize};
  for(int z=2; z<cellsPerAxis+1; z++) {
    CubeCell* priorCell = cell;
    cell = &cells[1][0][z];
    cell->lhl = priorCell->lhh;
    cell->hhl = priorCell->hhh;

    masses.emplace_back(cellCenter + Vector{-halfCell, halfCell, halfCell});
    cell->lhh = masses.size()-1;
    surfaceMassIndices.push_back(cell->lhh);

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMassIndices.push_back(cell->hhh);

    springs.emplace_back(cell->lhl, cell->lhh, k, c, cellSize);
    springs.emplace_back(cell->lhh, cell->hhh, k, c, cellSize);
    springs.emplace_back(cell->hhl, cell->hhh, k, c, cellSize);

    cellCenter[2] += cellSize;
  }

  cellCenter = firstCellCenter + Vector{cellSize, -cellSize, 0};
  for(int x=2; x<cellsPerAxis+1; x++) {
    CubeCell* priorCellX = &cells[x-1][0][1];
    CubeCell* priorCellZ = &cells[x][1][0];
    cell = &cells[x][0][1];
    cell->lhl = priorCellX->hhl;
    cell->lhh = priorCellX->hhh;
    cell->hhl = priorCellZ->hlh;

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMassIndices.push_back(cell->hhh);

    springs.emplace_back(cell->lhh, cell->hhh, k, c, cellSize);
    springs.emplace_back(cell->hhl, cell->hhh, k, c, cellSize);

    cellCenter[2] += cellSize;
    for(int z=2; z<cellsPerAxis+1; z++) {
      priorCellX = &cells[x-1][0][z];
      priorCellZ = cell;
      cell = &cells[x][0][z];
      cell->lhl = priorCellX->hhl;
      cell->lhh = priorCellX->hhh;
      cell->hhl = priorCellZ->hhh;

      masses.emplace_back(cellCenter + halfCell);
      cell->hhh = masses.size()-1;
      surfaceMassIndices.push_back(cell->hhh);

      springs.emplace_back(cell->lhh, cell->hhh, k, c, cellSize);
      springs.emplace_back(cell->hhl, cell->hhh, k, c, cellSize);

      cellCenter[2] += cellSize;
    }
    cellCenter[2] = firstCellCenter[2];
    cellCenter[0] += cellSize;
  }


  // Cells below x-axis minimum
  cell = &cells[0][1][1];
  cell->hll = cells[1][1][0].llh;
  cell->hhl = cells[1][1][0].lhh;
  cell->hlh = cells[1][0][1].lhh;

  massPos = firstCellCenter + Vector{-halfCell, halfCell, halfCell};
  masses.emplace_back(massPos);
  cell->hhh = masses.size()-1;
  surfaceMassIndices.push_back(cell->hhh);

  springs.emplace_back(cell->hlh, cell->hhh, k, c, cellSize);
  springs.emplace_back(cell->hhl, cell->hhh, k, c, cellSize);

  cellCenter = firstCellCenter + Vector{-cellSize, 0, cellSize};
  for(int z=2; z<cellsPerAxis+1; z++) {
    CubeCell* priorCell = cell;
    cell = &cells[0][1][z];
    cell->hll = priorCell->hlh;
    cell->hhl = priorCell->hhh;
    cell->hlh = cells[1][0][z].lhh;

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMassIndices.push_back(cell->hhh);

    springs.emplace_back(cell->hlh, cell->hhh, k, c, cellSize);
    springs.emplace_back(cell->hhl, cell->hhh, k, c, cellSize);

    cellCenter[2] += cellSize;
  }

  cellCenter = firstCellCenter + Vector{-cellSize, cellSize, 0};
  for(int y=2; y<cellsPerAxis+1; y++) {
    CubeCell* priorCellY = &cells[0][y-1][1];
    CubeCell* priorCellZ = &cells[1][y][0];
    cell = &cells[0][y][1];
    cell->hll = priorCellY->hhl;
    cell->hlh = priorCellY->hhh;
    cell->hhl = priorCellZ->lhh;

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMassIndices.push_back(cell->hhh);

    springs.emplace_back(cell->hlh, cell->hhh, k, c, cellSize);
    springs.emplace_back(cell->hhl, cell->hhh, k, c, cellSize);

    cellCenter[2] += cellSize;
    for(int z=2; z<cellsPerAxis+1; z++) {
      priorCellY = &cells[0][y-1][z];
      priorCellZ = cell;
      cell = &cells[0][y][z];
      cell->hll = priorCellY->hhl;
      cell->hlh = priorCellY->hhh;
      cell->hhl = priorCellZ->hhh;

      masses.emplace_back(cellCenter + halfCell);
      cell->hhh = masses.size()-1;
      surfaceMassIndices.push_back(cell->hhh);

      springs.emplace_back(cell->hlh, cell->hhh, k, c, cellSize);
      springs.emplace_back(cell->hhl, cell->hhh, k, c, cellSize);

      cellCenter[2] += cellSize;
    }
    cellCenter[2] = firstCellCenter[2];
    cellCenter[1] += cellSize;
  }

  /**
   * Fill in cube
   */

  cellCenter = firstCellCenter;
  for(int x=1; x<=cellsPerAxis; x++) {
    for(int y=1; y<=cellsPerAxis; y++) {
      for(int z=1; z<=cellsPerAxis; z++) {
        cells[x][y][z] = buildCell(cellCenter, cells[x-1][y][z], cells[x][y-1][z],
                                   cells[x][y][z-1], cellSize, k, c);
        if(x == cellsPerAxis || y == cellsPerAxis || z == cellsPerAxis)
          surfaceMassIndices.push_back(cells[x][y][z].hhh);

        // Connect center masses of adjacent cells
        // if(x != 1) {
        //   springs.emplace_back(cells[x-1][y][z].center, cells[x][y][z].center, k, c, cellSize);
        // }if(y != 1) {
        //   springs.emplace_back(cells[x][y-1][z].center, cells[x][y][z].center, k, c, cellSize);
        // }if(z != 1) {
        //   springs.emplace_back(cells[x][y][z-1].center, cells[x][y][z].center, k, c, cellSize);
        // }

        cellCenter[2] += cellSize;        // Increment cell z position 
      }
      cellCenter[2] = firstCellCenter[2]; // Reset cell z position
      cellCenter[1] += cellSize;          // Increment cell y position
    }
    cellCenter[1] = firstCellCenter[1];   // Reset cell y position
    cellCenter[0] += cellSize;            // Increment cell x position
  }




  // TEMP*******************************************************************************************
  //    - Move mass out of place to test soft body physics
  Vector temp = masses[0].getState();
  temp[Mass::POS] -= Vector(halfCell/2, 3);
  masses[0].update(temp);

  temp = masses[cells[cellsPerAxis][1][1].hll].getState();
  temp[Mass::POS] += Vector{halfCell, -halfCell, -halfCell};
  masses[cells[cellsPerAxis][1][1].hll].update(temp);




  /**
   * Initialize member variables
   */

  masses.shrink_to_fit();
  springs.shrink_to_fit();
  surfaceMassIndices.shrink_to_fit();
  surfaceMasses.resize(surfaceMassIndices.size());
  initSurfaceMasses();

  massRadii      = cellSize/10;
  boundingRadius = 1.2*vecNorm(Vector(sideLengths/2, 3));

  VecList state(masses.size());
  getState(state);
  initSolver(state);

  cornerMasses[0] = cells[1][1][1].lll;
  cornerMasses[1] = cells[1][1][cellsPerAxis].llh;
  cornerMasses[2] = cells[1][cellsPerAxis][1].lhl;
  cornerMasses[3] = cells[cellsPerAxis][1][1].hll;
  cornerMasses[4] = cells[1][cellsPerAxis][cellsPerAxis].lhh;
  cornerMasses[5] = cells[cellsPerAxis][1][cellsPerAxis].hll;
  cornerMasses[6] = cells[cellsPerAxis][cellsPerAxis][1].hhl;
  cornerMasses[7] = cells[cellsPerAxis][cellsPerAxis][cellsPerAxis].hhh;

  return buildMesh(cells, cellsPerAxis, material);
}
