#include "soft_cube.h"


/**
 * @brief Returns the center of mass which is approximated by taking the
 *        position of the mass located in the center of the cube.
 */
Vector SoftCube::getCenterOfMass() const {
  return masses[centerMass].getPos();
}


GLuint indexOf(const std::vector<int>& v, GLuint i) {
  return std::distance(v.begin(), std::find(v.begin(), v.end(), i));
}

/**
 * @brief  Duplicates a vertex in the vertex list to ensure edge vertices have
 *         separate normal vectors for each face they're a part of and updates
 *         the mass and vertex index lists to reflect the change.
 *
 * @param  vertices     List of display mesh vertex positions.
 * @param  indices      Vertex indices for display mesh.
 * @param  massIndices  Indices of masses that correspond to mesh vertices.
 * @param  i            Index of the vertex in the indices list to duplicate.
 */
void SoftCube::duplicateVertex(std::vector<GLfloat>& vertices, std::vector<GLuint>& indices,
                               std::vector<GLuint>& massIndices, GLuint i)
{
  massIndices.push_back(surfaceMasses[indices[i]]);
  vertices.insert(vertices.end(), {vertices[3*indices[i]],
                                   vertices[3*indices[i] + 1],
                                   vertices[3*indices[i] + 2]});
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
      indices.insert(indices.end(), {indexOf(surfaceMasses, cell->hhh),
                                     indexOf(surfaceMasses, cell->hll),
                                     indexOf(surfaceMasses, cell->hhl),
                                     indexOf(surfaceMasses, cell->hlh),
                                     indexOf(surfaceMasses, cell->hll),
                                     indexOf(surfaceMasses, cell->hhh)});
    }
  }

  // Triangles on y-axis min. and max. sides (front and back)
  for(int x=1; x<=cellsPerAxis; x++) {
    for(int z=1; z<=cellsPerAxis; z++) {
      cell = &cells[x][0][z];
      indices.insert(indices.end(), {cell->lhl, cell->hhl, cell->lhh,
                                     cell->lhh, cell->hhl, cell->hhh});

      cell = &cells[x][cellsPerAxis][z];
      indices.insert(indices.end(), {indexOf(surfaceMasses, cell->lhh),
                                     indexOf(surfaceMasses, cell->hhl),
                                     indexOf(surfaceMasses, cell->lhl),
                                     indexOf(surfaceMasses, cell->hhh),
                                     indexOf(surfaceMasses, cell->hhl),
                                     indexOf(surfaceMasses, cell->lhh)});

      // Duplicate edge vertices
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
      indices.insert(indices.end(), {indexOf(surfaceMasses, cell->hlh),
                                     indexOf(surfaceMasses, cell->lhh),
                                     indexOf(surfaceMasses, cell->llh),
                                     indexOf(surfaceMasses, cell->hhh),
                                     indexOf(surfaceMasses, cell->lhh),
                                     indexOf(surfaceMasses, cell->hlh)});

      // Duplicate edge vertices
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
  std::vector<GLuint>  massIndices;
  for(int i=0; i<surfaceMasses.size(); i++) {
    const Vector& state = masses[surfaceMasses[i]].getState();
    vertices.insert(vertices.end(), {(float)state[0], (float)state[1], (float)state[2]});
    massIndices.push_back(surfaceMasses[i]);
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
                             CubeCell& cellZ, double cellSize)
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

  // Length of springs connecting diagonally adjacent surface masses
  double diagSurfaceSpringLen = vecNorm(Vector(cellSize, 2));

  // Grid of cells storing positional data about mass indices
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
  surfaceMasses.push_back(cell->llh);

  massPos[1] += cellSize;
  masses.emplace_back(massPos);
  cell->lhh = masses.size()-1;
  surfaceMasses.push_back(cell->lhh);

  massPos += Vector{cellSize, -cellSize, 0};
  masses.emplace_back(massPos);
  cell->hlh = masses.size()-1;
  surfaceMasses.push_back(cell->hlh);

  massPos[1] += cellSize;
  masses.emplace_back(massPos);
  cell->hhh = masses.size()-1;
  surfaceMasses.push_back(cell->hhh);

  springs.emplace_back(cell->llh, cell->hhh, k, c, diagSurfaceSpringLen);
  springs.emplace_back(cell->lhh, cell->hlh, k, c, diagSurfaceSpringLen);

  cellCenter = firstCellCenter + Vector{0, cellSize, -cellSize};
  for(int y=2; y<cellsPerAxis+1; y++) {
    CubeCell* priorCell = cell;
    cell = &cells[1][y][0];
    cell->llh = priorCell->lhh;
    cell->hlh = priorCell->hhh;

    masses.emplace_back(cellCenter + Vector{-halfCell, halfCell, halfCell});
    cell->lhh = masses.size()-1;
    surfaceMasses.push_back(cell->lhh);

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

    springs.emplace_back(cell->llh, cell->hhh, k, c, diagSurfaceSpringLen);
    springs.emplace_back(cell->lhh, cell->hlh, k, c, diagSurfaceSpringLen);

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
    surfaceMasses.push_back(cell->hlh);

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

    springs.emplace_back(cell->llh, cell->hhh, k, c, diagSurfaceSpringLen);
    springs.emplace_back(cell->lhh, cell->hlh, k, c, diagSurfaceSpringLen);

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
      surfaceMasses.push_back(cell->hhh);

      springs.emplace_back(cell->llh, cell->hhh, k, c, diagSurfaceSpringLen);
      springs.emplace_back(cell->lhh, cell->hlh, k, c, diagSurfaceSpringLen);

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
  surfaceMasses.push_back(cell->lhh);

  massPos[0] += cellSize;
  masses.emplace_back(massPos);
  cell->hhh = masses.size()-1;
  surfaceMasses.push_back(cell->hhh);

  springs.emplace_back(cell->lhl, cell->hhh, k, c, diagSurfaceSpringLen);
  springs.emplace_back(cell->lhh, cell->hhl, k, c, diagSurfaceSpringLen);

  cellCenter = firstCellCenter + Vector{0, -cellSize, cellSize};
  for(int z=2; z<cellsPerAxis+1; z++) {
    CubeCell* priorCell = cell;
    cell = &cells[1][0][z];
    cell->lhl = priorCell->lhh;
    cell->hhl = priorCell->hhh;

    masses.emplace_back(cellCenter + Vector{-halfCell, halfCell, halfCell});
    cell->lhh = masses.size()-1;
    surfaceMasses.push_back(cell->lhh);

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

    springs.emplace_back(cell->lhl, cell->hhh, k, c, diagSurfaceSpringLen);
    springs.emplace_back(cell->lhh, cell->hhl, k, c, diagSurfaceSpringLen);

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
    surfaceMasses.push_back(cell->hhh);

    springs.emplace_back(cell->lhl, cell->hhh, k, c, diagSurfaceSpringLen);
    springs.emplace_back(cell->lhh, cell->hhl, k, c, diagSurfaceSpringLen);

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
      surfaceMasses.push_back(cell->hhh);

      springs.emplace_back(cell->lhl, cell->hhh, k, c, diagSurfaceSpringLen);
      springs.emplace_back(cell->lhh, cell->hhl, k, c, diagSurfaceSpringLen);

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
  surfaceMasses.push_back(cell->hhh);

  springs.emplace_back(cell->hll, cell->hhh, k, c, diagSurfaceSpringLen);
  springs.emplace_back(cell->hlh, cell->hhl, k, c, diagSurfaceSpringLen);

  cellCenter = firstCellCenter + Vector{-cellSize, 0, cellSize};
  for(int z=2; z<cellsPerAxis+1; z++) {
    CubeCell* priorCell = cell;
    cell = &cells[0][1][z];
    cell->hll = priorCell->hlh;
    cell->hhl = priorCell->hhh;
    cell->hlh = cells[1][0][z].lhh;

    masses.emplace_back(cellCenter + halfCell);
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

    springs.emplace_back(cell->hll, cell->hhh, k, c, diagSurfaceSpringLen);
    springs.emplace_back(cell->hlh, cell->hhl, k, c, diagSurfaceSpringLen);

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
    surfaceMasses.push_back(cell->hhh);

    springs.emplace_back(cell->hll, cell->hhh, k, c, diagSurfaceSpringLen);
    springs.emplace_back(cell->hlh, cell->hhl, k, c, diagSurfaceSpringLen);

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
      surfaceMasses.push_back(cell->hhh);

      springs.emplace_back(cell->hll, cell->hhh, k, c, diagSurfaceSpringLen);
      springs.emplace_back(cell->hlh, cell->hhl, k, c, diagSurfaceSpringLen);

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
                                   cells[x][y][z-1], cellSize);
        if(x == cellsPerAxis || y == cellsPerAxis || z == cellsPerAxis)
          surfaceMasses.push_back(cells[x][y][z].hhh);

        // Connect diagonal surface masses
        if(x == cellsPerAxis) {
          springs.emplace_back(cells[x][y][z].hll, cells[x][y][z].hhh, k, c, diagSurfaceSpringLen);
          springs.emplace_back(cells[x][y][z].hlh, cells[x][y][z].hhl, k, c, diagSurfaceSpringLen);
        }
        if(y == cellsPerAxis) {
          springs.emplace_back(cells[x][y][z].lhl, cells[x][y][z].hhh, k, c, diagSurfaceSpringLen);
          springs.emplace_back(cells[x][y][z].lhh, cells[x][y][z].hhl, k, c, diagSurfaceSpringLen);
        }
        if(z == cellsPerAxis) {
          springs.emplace_back(cells[x][y][z].llh, cells[x][y][z].hhh, k, c, diagSurfaceSpringLen);
          springs.emplace_back(cells[x][y][z].lhh, cells[x][y][z].hlh, k, c, diagSurfaceSpringLen);
        }

        cellCenter[2] += cellSize;        // Increment cell z position 
      }
      cellCenter[2] = firstCellCenter[2]; // Reset cell z position
      cellCenter[1] += cellSize;          // Increment cell y position
    }
    cellCenter[1] = firstCellCenter[1];   // Reset cell y position
    cellCenter[0] += cellSize;            // Increment cell x position
  }

  // Create springs
  subdivideCell(cells, cellsPerAxis, k, c);


  /**
   * Initialize member variables
   */

  int centralCell = cellsPerAxis/2;
  if(cellsPerAxis%2 == 0)
    centerMass = cells[centralCell][centralCell][centralCell].lll;
  else
    centerMass = cells[centralCell][centralCell][centralCell].center;

  boundingRadius = 1.2*vecNorm(Vector(sideLengths/2, 3));
  masses.shrink_to_fit();
  springs.shrink_to_fit();
  surfaceMasses.shrink_to_fit();

  VecList state(masses.size());
  getState(state);
  initSolver(state);

  return buildMesh(cells, cellsPerAxis, material);
}


/**
 * @brief  Creates the springs to connect the masses of a cell, which may be a
 *         super-cell comprised of multiple individual cells, and recursively
 *         subdivides that cell until it has created the springs for each
 *         individual cell within the super-cell. The result is a fractal of
 *         cells with each subsequent smaller level having half the cells per
 *         axis as the last (just under half for cells with an odd number of
 *         cells per axis).
 *
 * @param  cells         Cell grid.
 * @param  cellsPerAxis  Number of cells per axis of the super-cell.
 * @param  x             Starting x-axis cell grid index.
 * @param  y             Starting y-axis cell grid index.
 * @param  z             Starting z-axis cell grid index.
 * @param  k             Spring coefficient.
 * @param  c             Damping coefficient.
 */
void SoftCube::subdivideCell(CubeCellGrid& cells, int cellsPerAxis, double k, double c,
                              int x, int y, int z)
{
  if(cellsPerAxis < 1)
    return;

  // Find masses of cell
  int span = cellsPerAxis - 1;
  int center;
  if(cellsPerAxis & 1) center = cells[x+span/2][y+span/2][z+span/2].center;
  else                 center = cells[x+span/2][y+span/2][z+span/2].hhh;

  int lll = cells[x][y][z].lll;
  int llh = cells[x][y][z+span].llh;
  int lhl = cells[x][y+span][z].lhl;
  int lhh = cells[x][y+span][z+span].lhh;
  int hll = cells[x+span][y][z].hll;
  int hlh = cells[x+span][y][z+span].hlh;
  int hhl = cells[x+span][y+span][z].hhl;
  int hhh = cells[x+span][y+span][z+span].hhh;

  // Connect adjacent cell vertices
  double cellSize = vecNorm(masses[lll].getPos() - masses[llh].getPos());
  addSpring(lll, llh, k, c, cellSize);
  addSpring(lll, lhl, k, c, cellSize);
  addSpring(lll, hll, k, c, cellSize);
  addSpring(llh, lhh, k, c, cellSize);
  addSpring(llh, hlh, k, c, cellSize);
  addSpring(lhl, lhh, k, c, cellSize);
  addSpring(lhl, hhl, k, c, cellSize);
  addSpring(lhh, hhh, k, c, cellSize);
  addSpring(hll, hlh, k, c, cellSize);
  addSpring(hll, hhl, k, c, cellSize);
  addSpring(hlh, hhh, k, c, cellSize);
  addSpring(hhl, hhh, k, c, cellSize);

  // Connect each vertex to central mass
  double vertexDist = vecNorm(Vector(cellSize/2, 3));
  springs.emplace_back(center, lll, k, c, vertexDist);
  springs.emplace_back(center, llh, k, c, vertexDist);
  springs.emplace_back(center, lhl, k, c, vertexDist);
  springs.emplace_back(center, lhh, k, c, vertexDist);
  springs.emplace_back(center, hll, k, c, vertexDist);
  springs.emplace_back(center, hlh, k, c, vertexDist);
  springs.emplace_back(center, hhl, k, c, vertexDist);
  springs.emplace_back(center, hhh, k, c, vertexDist);

  // Subdivide cell
  double offset = ceil(cellsPerAxis/2.0);
  if(cellsPerAxis > 1) {
    subdivideCell(cells, cellsPerAxis/2, k, c, x,        y,        z);
    subdivideCell(cells, cellsPerAxis/2, k, c, x+offset, y,        z);
    subdivideCell(cells, cellsPerAxis/2, k, c, x,        y+offset, z);
    subdivideCell(cells, cellsPerAxis/2, k, c, x,        y,        z+offset);
    subdivideCell(cells, cellsPerAxis/2, k, c, x+offset, y+offset, z);
    subdivideCell(cells, cellsPerAxis/2, k, c, x+offset, y,        z+offset);
    subdivideCell(cells, cellsPerAxis/2, k, c, x,        y+offset, z+offset);
    subdivideCell(cells, cellsPerAxis/2, k, c, x+offset, y+offset, z+offset);

    // Account for middle cells that get skipped with odd numbered cells
    if(cellsPerAxis & 1) {
      offset = cellsPerAxis/2;
      for(int i=0; i<cellsPerAxis; i++) {
        if(i == offset) {
          subdivideCell(cells, 1, k, c, x+offset, y+offset, z+offset);
          continue;
        }
        subdivideCell(cells, 1, k, c, x+i, y+offset, z+offset);
        subdivideCell(cells, 1, k, c, x+offset, y+i, z+offset);
        subdivideCell(cells, 1, k, c, x+offset, y+offset, z+i);
      }

      for(int i=0; i<cellsPerAxis; i++) {
        if(i == offset) continue;
        for(int j=0; j<cellsPerAxis; j++) {
          if(j == offset) continue;
          subdivideCell(cells, 1, k, c, x+i, y+j, z+offset);
          subdivideCell(cells, 1, k, c, x+i, y+offset, z+j);
          subdivideCell(cells, 1, k, c, x+offset, y+i, z+j);
        }
      }
    }
  }
}


/**
 * @brief  Adds a spring to the list only if a spring connecting the two masses
 *         doesn't already exist.
 *
 * @param  mass1    First mass connected to spring.
 * @param  mass2    Second mass connected to spring.
 * @param  k        Spring coefficient.
 * @param  c        Damping coefficient.
 * @param  restLen  Rest length of the spring.
 */
void SoftCube::addSpring(int mass1, int mass2, double k, double c, double length) {
  for(const Spring& spring : springs) {
    const std::pair<int, int>& springMasses = spring.getMassIndices();
    if((mass1 == springMasses.first || mass1 == springMasses.second) &&
       (mass2 == springMasses.first || mass2 == springMasses.second))
    {
      return;
    }
  }
  springs.emplace_back(mass1, mass2, k, c, length);
}
