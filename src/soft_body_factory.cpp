#include "soft_body_factory.h"


unsigned int indexOf(const std::vector<int>& v, unsigned int i) {
  return std::distance(v.begin(), std::find(v.begin(), v.end(), i));
}

std::vector<GLuint> SoftBodyFactory::getCubeVertexIndices(const std::vector<int>& surfaceMasses,
                         int cellsPerAxis, std::vector<std::vector<std::vector<CubeCell>>>& cells)
{
  std::vector<GLuint> indices;
  CubeCell* cell;

  // Triangles on x-axis min. and max. sides (left and right)
  for(int y=1; y<=cellsPerAxis; y++) {
    for(int z=1; z<=cellsPerAxis; z++) {
      cell = &cells[0][y][z];
      indices.insert(indices.end(), {cell->hhl, cell->hll, cell->hhh,
                                     cell->hhh, cell->hll, cell->hlh});

      cell = &cells[cellsPerAxis][y][z];
      indices.push_back(indexOf(surfaceMasses, cell->hhh));
      indices.push_back(indexOf(surfaceMasses, cell->hll));
      indices.push_back(indexOf(surfaceMasses, cell->hhl));
      indices.push_back(indexOf(surfaceMasses, cell->hlh));
      indices.push_back(indexOf(surfaceMasses, cell->hll));
      indices.push_back(indexOf(surfaceMasses, cell->hhh));
    }
  }
  
  // Triangles on y-axis min. and max. sides (front and back)
  for(int x=1; x<=cellsPerAxis; x++) {
    for(int z=1; z<=cellsPerAxis; z++) {
      cell = &cells[x][0][z];
      indices.insert(indices.end(), {cell->lhl, cell->hhl, cell->lhh,
                                     cell->lhh, cell->hhl, cell->hhh});

      cell = &cells[x][cellsPerAxis][z];
      indices.push_back(indexOf(surfaceMasses, cell->lhh));
      indices.push_back(indexOf(surfaceMasses, cell->hhl));
      indices.push_back(indexOf(surfaceMasses, cell->lhl));
      indices.push_back(indexOf(surfaceMasses, cell->hhh));
      indices.push_back(indexOf(surfaceMasses, cell->hhl));
      indices.push_back(indexOf(surfaceMasses, cell->lhh));
    }
  }

  // Triangles on z-axis min. and max. sides (bottom and top)
  for(int x=1; x<=cellsPerAxis; x++) {
    for(int y=1; y<=cellsPerAxis; y++) {
      cell = &cells[x][y][0];
      indices.insert(indices.end(), {cell->llh, cell->lhh, cell->hlh,
                                     cell->hlh, cell->lhh, cell->hhh});

      cell = &cells[x][y][cellsPerAxis];
      indices.push_back(indexOf(surfaceMasses, cell->hlh));
      indices.push_back(indexOf(surfaceMasses, cell->lhh));
      indices.push_back(indexOf(surfaceMasses, cell->llh));
      indices.push_back(indexOf(surfaceMasses, cell->hhh));
      indices.push_back(indexOf(surfaceMasses, cell->lhh));
      indices.push_back(indexOf(surfaceMasses, cell->hlh));
    }
  }

  return indices;
}


Mesh SoftBodyFactory::buildCubeMesh(const std::vector<int>& surfaceMasses, const SoftBody& cube,
                               std::vector<std::vector<std::vector<CubeCell>>>& cells, int numCells)
{
  // Get vertex positions
  const std::vector<Mass*>& masses = cube.getSurfaceMasses();
  int cellsPerAxis = cbrt(numCells);
  int numV = surfaceMasses.size() * 3;
  GLfloat vertices[numV];
  GLfloat normals[numV];
  for(int i=0; i<surfaceMasses.size(); i++) {
    Vector pos = masses[i]->getPos();
    vertices[i*3]     = pos[0];
    vertices[i*3 + 1] = pos[1];
    vertices[i*3 + 2] = pos[2];
  }

  // Get vertex indices
  std::vector<GLuint> indices = getCubeVertexIndices(surfaceMasses, cellsPerAxis, cells);

  return Mesh(vertices, normals, indices.data(), numV, indices.size());
}


CubeCell SoftBodyFactory::buildCubeCell(std::vector<Mass>& masses, Vector& cellCenter,
                                  float cellSize, CubeCell& cellX, CubeCell& cellY, CubeCell& cellZ)
{
  CubeCell cell;

  // Create new masses
  masses.emplace_back(cellCenter);
  cell.center = masses.size()-1;
  masses.emplace_back(cellCenter + cellSize/2);
  cell.hhh = masses.size()-1;

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
 * @brief Builds the mass-spring structure for a cube out of 2x2x2 cells of
 *        masses with one more mass in the center. The given number of cells is
 *        rounded to the nearest cubic number.
 * @param position Position of the center of the cube.
 * @param size Side length of the cube.
 * @param numCells Number of cells in the cube.
 * @param k Spring coefficient.
 * @param c Damping coefficient.
 * @return Pair including the cube and a mesh to display it.
 */
std::pair<SoftBody, Mesh> SoftBodyFactory::buildCube(Vector position, float size,
                                                     unsigned int numCells, float k, float c)
{ 
  std::vector<Mass> masses;
  std::vector<Spring> springs;
  std::vector<int> surfaceMasses;

  // Round number of cells to nearest cube
  numCells = pow(round(cbrt(numCells)), 3);
  int cellsPerAxis = cbrt(numCells);

  float cellSize = size / cellsPerAxis;                     // Cell side length
  float halfCellSize = cellSize/2;
  float vertexDist = vecNorm(Vector(size/2, 3));            // Distance from center to vertices
  Vector firstCellCenter = position - size/2 + halfCellSize;
  Vector cellCenter;                                        // Center of current cell
  
  std::vector<std::vector<std::vector<CubeCell>>> cells;
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
  Vector massPos = firstCellCenter - halfCellSize;
  masses.emplace_back(massPos);
  // masses.push_back(Mass(massPos));
  cell->llh = masses.size()-1;
  surfaceMasses.push_back(cell->llh);

  massPos[1] += cellSize;
  masses.emplace_back(massPos);
  // masses.push_back(Mass(massPos));
  cell->lhh = masses.size()-1;
  surfaceMasses.push_back(cell->lhh);

  massPos += Vector{cellSize, -cellSize, 0};
  masses.emplace_back(massPos);
  // masses.push_back(Mass(massPos));
  cell->hlh = masses.size()-1;
  surfaceMasses.push_back(cell->hlh);

  massPos[1] += cellSize;
  masses.emplace_back(massPos);
  // masses.push_back(Mass(massPos));
  cell->hhh =  masses.size()-1;
  surfaceMasses.push_back(cell->hhh);

  // First y-axis row
  cellCenter = firstCellCenter + Vector{0, cellSize, -cellSize};
  for(int y=2; y<cellsPerAxis+1; y++) {
    CubeCell* priorCell = cell;
    cell = &cells[1][y][0];
    cell->llh = priorCell->lhh;
    cell->hlh = priorCell->hhh;

    masses.emplace_back(cellCenter + Vector{-halfCellSize, halfCellSize, halfCellSize});
    // masses.push_back(Mass(cellCenter + Vector{-halfCellSize, halfCellSize, halfCellSize}));
    cell->lhh = masses.size()-1;
    surfaceMasses.push_back(cell->lhh);

    masses.emplace_back(cellCenter + halfCellSize);
    // masses.push_back(Mass(cellCenter + halfCellSize));
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

    cellCenter[1] += cellSize;
  }

  // 
  cellCenter = firstCellCenter + Vector{cellSize, 0, -cellSize};
  for(int x=2; x<cellsPerAxis+1; x++) {
    CubeCell* priorCellX = &cells[x-1][1][0];
    cell = &cells[x][1][0];
    cell->llh = priorCellX->hlh;
    cell->lhh = priorCellX->hhh;

    masses.emplace_back(cellCenter + Vector{halfCellSize, -halfCellSize, halfCellSize});
    // masses.push_back(Mass(cellCenter + Vector{halfCellSize, -halfCellSize, halfCellSize}));
    cell->hlh = masses.size()-1;
    surfaceMasses.push_back(cell->hlh);

    masses.emplace_back(cellCenter + halfCellSize);
    // masses.push_back(Mass(cellCenter + halfCellSize));
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

    cellCenter[1] += cellSize;
    for(int y=2; y<cellsPerAxis+1; y++) {
      priorCellX = &cells[x-1][y][0];
      CubeCell* priorCellY = cell;
      cell = &cells[x][y][0];
      cell->llh = priorCellX->hlh;
      cell->lhh = priorCellX->hhh;
      cell->hlh = priorCellY->hhh;

      masses.emplace_back(cellCenter + halfCellSize);
      cell->hhh = masses.size()-1;
      surfaceMasses.push_back(cell->hhh);

      cellCenter[1] += cellSize;
    }
    cellCenter[1] = firstCellCenter[1];
    cellCenter[0] += cellSize;
  }


  // Cells below y-axis minimum
  cell = &cells[1][0][1];
  cell->lhl = cells[1][1][0].llh;
  cell->hhl = cells[1][1][0].hlh;

  massPos = firstCellCenter + Vector{-halfCellSize, halfCellSize, halfCellSize};
  masses.emplace_back(massPos);
  cell->lhh = masses.size()-1;
  surfaceMasses.push_back(cell->lhh);

  massPos[0] += cellSize;
  masses.emplace_back(massPos);
  cell->hhh = masses.size()-1;
  surfaceMasses.push_back(cell->hhh);

  cellCenter = firstCellCenter + Vector{0, -cellSize, cellSize};
  for(int z=2; z<cellsPerAxis+1; z++) {
    CubeCell* priorCell = cell;
    cell = &cells[1][0][z];
    cell->lhl = priorCell->lhh;
    cell->hhl = priorCell->hhh;

    masses.emplace_back(cellCenter + Vector{-halfCellSize, halfCellSize, halfCellSize});
    cell->lhh = masses.size()-1;
    surfaceMasses.push_back(cell->lhh);

    masses.emplace_back(cellCenter + halfCellSize);
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

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

    masses.emplace_back(cellCenter + halfCellSize);
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

    cellCenter[2] += cellSize;
    for(int z=2; z<cellsPerAxis+1; z++) {
      priorCellX = &cells[x-1][0][z];
      priorCellZ = cell;
      cell = &cells[x][0][z];
      cell->lhl = priorCellX->hhl;
      cell->lhh = priorCellX->hhh;
      cell->hhl = priorCellZ->hhh;

      masses.emplace_back(cellCenter + halfCellSize);
      cell->hhh = masses.size()-1;
      surfaceMasses.push_back(cell->hhh);

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

  massPos = firstCellCenter + Vector{-halfCellSize, halfCellSize, halfCellSize};
  masses.emplace_back(massPos);
  cell->hhh = masses.size()-1;
  surfaceMasses.push_back(cell->hhh);

  cellCenter = firstCellCenter + Vector{-cellSize, 0, cellSize};
  for(int z=2; z<cellsPerAxis+1; z++) {
    CubeCell* priorCell = cell;
    cell = &cells[0][1][z];
    cell->hll = priorCell->hlh;
    cell->hhl = priorCell->hhh;
    cell->hlh = cells[1][0][z].lhh;

    masses.emplace_back(cellCenter + halfCellSize);
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

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

    masses.emplace_back(cellCenter + halfCellSize);
    cell->hhh = masses.size()-1;
    surfaceMasses.push_back(cell->hhh);

    cellCenter[2] += cellSize;
    for(int z=2; z<cellsPerAxis+1; z++) {
      priorCellY = &cells[0][y-1][z];
      priorCellZ = cell;
      cell = &cells[0][y][z];
      cell->hll = priorCellY->hhl;
      cell->hlh = priorCellY->hhh;
      cell->hhl = priorCellZ->hhh;

      masses.emplace_back(cellCenter + halfCellSize);
      cell->hhh = masses.size()-1;
      surfaceMasses.push_back(cell->hhh);

      cellCenter[2] += cellSize;
    }
    cellCenter[2] = firstCellCenter[2];
    cellCenter[1] += cellSize;
  }


  /**
   * Create cube
   */
  cellCenter = firstCellCenter;
  for(int x=1; x<=cellsPerAxis; x++) {
    for(int y=1; y<=cellsPerAxis; y++) {
      for(int z=1; z<=cellsPerAxis; z++) {
        cells[x][y][z] = buildCubeCell(masses, cellCenter, cellSize, cells[x-1][y][z],
                                       cells[x][y-1][z], cells[x][y][z-1]);
        if(x == cellsPerAxis || y == cellsPerAxis || z == cellsPerAxis)
          surfaceMasses.push_back(cells[x][y][z].hhh);
        cellCenter[2] += cellSize;        // Increment cell z position 
      }
      cellCenter[1] += cellSize;          // Increment cell y position
      cellCenter[2] = firstCellCenter[2]; // Reset cell z position
    }
    cellCenter[0] += cellSize;            // Increment cell x position
    cellCenter[1] = firstCellCenter[1];   // Reset cell y position
  }

  SoftBody cube(masses, springs, surfaceMasses);
  return std::pair<SoftBody, Mesh>(cube, buildCubeMesh(surfaceMasses, cube, cells, numCells));
























//   for(int x=0; x<cellsPerAxis; x++) {
//     for(int y=0; y<cellsPerAxis; y++) {
//       for(int z=0; z<cellsPerAxis; z++) {
//         CubeCell& cell = cells[x][y][z];
// 
//         // Create cell masses where necessary
//         masses.push_back(Mass(cellCenter));
//         cell.center = {&masses.back(), masses.size()-1};
// 
//         if(x == 0) {
//           if(y == 0) {
//             if(z == 0) {
//               masses.push_back(Mass(cellCenter - Vector(vertexDist, 3)));
//               surfaceMasses.push_back(&masses.back());
//               cell.lll = {&masses.back(), masses.size()-1};
//             }
//             masses.push_back(Mass(cellCenter + Vector{-vertexDist, -vertexDist, vertexDist}));
//             surfaceMasses.push_back(&masses.back());
//             cell.llh = {&masses.back(), masses.size()-1};
//           }
// 
//           if(z == 0) {
//             masses.push_back(Mass(cellCenter + Vector{-vertexDist, vertexDist, -vertexDist}));
//             surfaceMasses.push_back(&masses.back());
//             cell.lhl = {&masses.back(), masses.size()-1};
//           }
//           masses.push_back(Mass(cellCenter + Vector{-vertexDist, vertexDist, vertexDist}));
//           surfaceMasses.push_back(&masses.back());
//           cell.lhh = {&masses.back(), masses.size()-1};
//         }
//         
//         if(y == 0) {
//           if(z == 0) {
//             masses.push_back(Mass(cellCenter + Vector{vertexDist, -vertexDist, -vertexDist}));
//             surfaceMasses.push_back(&masses.back());
//             cell.hll = {&masses.back(), masses.size()-1};
//           }
//           masses.push_back(Mass(cellCenter + Vector{vertexDist, -vertexDist, vertexDist}));
//           surfaceMasses.push_back(&masses.back());
//           cell.hlh = {&masses.back(), masses.size()-1};
//         }
// 
//         if(z == 0) {
//           masses.push_back(Mass(cellCenter + Vector{vertexDist, vertexDist, -vertexDist}));
//           surfaceMasses.push_back(&masses.back());
//           cell.hhl = {&masses.back(), masses.size()-1};
//         }
// 
//         masses.push_back(Mass(cellCenter + Vector(vertexDist, 3)));
//         cell.hhh = {&masses.back(), masses.size()-1};
//         if(x == cellsPerAxis-1 || y == cellsPerAxis-1)
//           surfaceMasses.push_back(&masses.back());
//         
//         // Connect to existing cells
//         if(cell.lll.index == -1) cell.lll = cells[x][y][z-1].llh;
//         if(cell.llh.index == -1) cell.llh = cells[x][y-1][z].lhh;
//         if(cell.lhl.index == -1) cell.lhl = cells[x][y][z-1].lhh;
//         if(cell.lhh.index == -1) cell.lhh = cells[x-1][y][z].hhh;
// 
//         if(cell.hll.index == -1) cell.hll = cells[x-1][y][z].hhh;
// 
// 
// 
// 
// 
//
//
//
// 
// 
// 
//         int xSurface=0, ySurface=0, zSurface=0;
// 
//         // Connect to existing cells
//         if(x > 0) {
//           CubeCell& cellX = cells[x-1][y][z];
//           if(cellX.center.index != -1) {
//             cell.lll = cellX.hll;
//             cell.llh = cellX.hlh;
//             cell.lhl = cellX.hhl;
//             cell.lhh = cellX.hhh;
//           }
//         }
//         else if(x == cellsPerAxis-1) xSurface = 1;
//         else xSurface = -1;
// 
//         if(y > 0) {
//           CubeCell& cellY = cells[x][y-1][z];
//           if(cellY.center.index != -1) {
//             cell.lll = cellY.lhl;
//             cell.llh = cellY.lhh;
//             cell.hll = cellY.hhl;
//             cell.hlh = cellY.hhh;
//           }
//         }
//         else if(y == cellsPerAxis-1) ySurface = 1;
//         else ySurface = -1;
// 
//         if(z > 0) {
//           CubeCell& cellZ = cells[x][y][z-1];
//           if(cellZ.center.index != -1) {
//             cell.lll = cellZ.llh;
//             cell.lhl = cellZ.lhh;
//             cell.hll = cellZ.hlh;
//             cell.hhl = cellZ.hhh;
//           }
//         } 
//         else if(z == cellsPerAxis-1) zSurface = 1;
//         else zSurface = -1;
// 
//         // Create new masses and springs (PROBABLY GOING IN SEPARATE FUNCTION)
//         masses.push_back(Mass(cellCenter));
//         cell.center = {&masses.back(), masses.size()-1};
//         if(cell.lll.index == -1) {
//           masses.push_back(Mass(cellCenter - Vector(vertexDist, 3)));
//           cell.lll = {&masses.back(), masses.size()-1};
//           springs.push_back(Spring(cell.center.index, cell.lll.index, k, c, vertexDist));
//         }if(cell.llh.index == -1) {
//           masses.push_back(Mass(cellCenter + Vector{-vertexDist, -vertexDist, vertexDist}));
//           cell.llh = {&masses.back(), masses.size()-1};
//           springs.push_back(Spring(cell.center.index, cell.llh.index, k, c, vertexDist));
//           springs.push_back(Spring(cell.lll.index,    cell.llh.index, k, c, cellSize));
//         }if(cell.lhl.index == -1) {
//           masses.push_back(Mass(cellCenter + Vector{-vertexDist, vertexDist, -vertexDist}));
//           cell.lhl = {&masses.back(), masses.size()-1};
//           springs.push_back(Spring(cell.center.index, cell.lhl.index, k, c, vertexDist));
//           springs.push_back(Spring(cell.lll.index,    cell.lhl.index, k, c, cellSize));
//         }if(cell.lhh.index == -1) {
//           masses.push_back(Mass(cellCenter + Vector{-vertexDist, vertexDist, vertexDist}));
//           cell.lhh = {&masses.back(), masses.size()-1};
//           springs.push_back(Spring(cell.center.index, cell.lhh.index, k, c, vertexDist));
//           springs.push_back(Spring(cell.llh.index,    cell.lhh.index, k, c, cellSize));
//           springs.push_back(Spring(cell.lhl.index,    cell.lhh.index, k, c, cellSize));
//         }if(cell.hll.index == -1) {
//           masses.push_back(Mass(cellCenter + Vector{vertexDist, -vertexDist, -vertexDist}));
//           cell.hll = {&masses.back(), masses.size()-1};
//           springs.push_back(Spring(cell.center.index, cell.hll.index, k, c, vertexDist));
//           springs.push_back(Spring(cell.lll.index,    cell.lhl.index, k, c, cellSize));
//         }if(cell.hlh.index == -1) {
//           masses.push_back(Mass(cellCenter + Vector{vertexDist, -vertexDist, vertexDist}));
//           cell.hlh = {&masses.back(), masses.size()-1};
//           springs.push_back(Spring(cell.center.index, cell.hlh.index, k, c, vertexDist));
//           springs.push_back(Spring(cell.llh.index,    cell.hlh.index, k, c, cellSize));
//           springs.push_back(Spring(cell.hll.index,    cell.hlh.index, k, c, cellSize));
//         }if(cell.hhl.index == -1) {
//           masses.push_back(Mass(cellCenter + Vector{vertexDist, vertexDist, -vertexDist}));
//           cell.hhl = {&masses.back(), masses.size()-1};
//           springs.push_back(Spring(cell.center.index, cell.hhl.index, k, c, vertexDist));
//           springs.push_back(Spring(cell.hhl.index,    cell.lhl.index, k, c, cellSize));
//           springs.push_back(Spring(cell.hhl.index,    cell.hll.index, k, c, cellSize));
//         }if(cell.hhh.index == -1) {
//           masses.push_back(Mass(cellCenter + Vector(vertexDist, 3)));
//           cell.hhh = {&masses.back(), masses.size()-1};
//           springs.push_back(Spring(cell.center.index, cell.hhh.index, k, c, vertexDist));
//           springs.push_back(Spring(cell.hhh.index,    cell.lhh.index, k, c, cellSize));
//           springs.push_back(Spring(cell.hhh.index,    cell.hlh.index, k, c, cellSize));
//           springs.push_back(Spring(cell.hhh.index,    cell.hhl.index, k, c, cellSize));
//         }
// 
//         // Get surface masses (PROBABLY GOING IN SEPARATE FUNCTION (which also makes the mesh index array))
// 
//         // TODO:
//         //   - need to check if masses had already been added to surface masses list
//         //   - finish positive z surface (don't forget to change the 'lll' type names)
//         
//         if(xSurface == -1) {
//           surfaceMasses.push_back(cell.lll.mass);
//           surfaceMasses.push_back(cell.llh.mass);
//           surfaceMasses.push_back(cell.lhl.mass);
//           surfaceMasses.push_back(cell.lhh.mass);
//         }else if(xSurface == 1) {
//           surfaceMasses.push_back(cell.hll.mass);
//           surfaceMasses.push_back(cell.hlh.mass);
//           surfaceMasses.push_back(cell.hhl.mass);
//           surfaceMasses.push_back(cell.hhh.mass);
//         }
//         if(ySurface == -1) {
//           if(xSurface != -1) {
//             surfaceMasses.push_back(cell.lll.mass);
//             surfaceMasses.push_back(cell.llh.mass);
//           }if(xSurface != 1) {
//             surfaceMasses.push_back(cell.hll.mass);
//             surfaceMasses.push_back(cell.hlh.mass);
//           }
//         }else if(ySurface == 1) {
//           if(xSurface != -1) {
//             surfaceMasses.push_back(cell.lhl.mass);
//             surfaceMasses.push_back(cell.lhh.mass);
//           }if(xSurface != 1) {
//             surfaceMasses.push_back(cell.hhl.mass);
//             surfaceMasses.push_back(cell.hhh.mass);
//           }
//         }
//         if(zSurface == -1) {
//           if(xSurface != -1) {
//             if(ySurface != -1) surfaceMasses.push_back(cell.lll.mass);
//             if(ySurface !=  1) surfaceMasses.push_back(cell.lhl.mass);
//           }if(xSurface != 1) {
//             if(ySurface != -1) surfaceMasses.push_back(cell.hll.mass);
//             if(ySurface !=  1) surfaceMasses.push_back(cell.hhl.mass);
//           }
//         }else if(zSurface == 1) {
//           if(xSurface != -1) {
//             surfaceMasses.push_back(cell.lhl.mass);
//             surfaceMasses.push_back(cell.lhh.mass);
//           }if(xSurface != 1) {
//             surfaceMasses.push_back(cell.hhl.mass);
//             surfaceMasses.push_back(cell.hhh.mass);
//           }
//         }
// 
//         cellCenter[2] += cellSize;        // Increment cell z position 
//       }
//       cellCenter[1] += cellSize;          // Increment cell y position
//       cellCenter[2] = firstCellCenter[2]; // Reset cell z position
//     }
//     cellCenter[0] += cellSize;            // Increment cell x position
//     cellCenter[1] = firstCellCenter[1];   // Reset cell y position
//   }
}
