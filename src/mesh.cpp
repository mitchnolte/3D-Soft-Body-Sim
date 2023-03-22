#include "mesh.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


Mesh::Mesh() {}

Mesh::Mesh(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI,
           const Material& material)
{
  loadVertexData(vertices, normals, indices, numV, numI);
  setMaterial(material);
}


/**
 * @brief Calculates the normal vector of a triangle with the 3 given points.
 * @param normal Ouput array.
 */
void Mesh::triangleNormal(GLfloat normal[], GLfloat p1[3], GLfloat p2[3], GLfloat p3[3]) {
  GLfloat u[3] = {p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]};
  GLfloat v[3] = {p3[0]-p2[0], p3[1]-p2[1], p3[2]-p2[2]};

  // Cross product
  normal[0] = (u[1]*v[2] - u[2]*v[1]);
  normal[1] = (u[2]*v[0] - u[0]*v[2]),
  normal[2] = (u[0]*v[1] - u[1]*v[0]);
}


/**
 * @brief Computes the normal vectors for a mesh.
 * @param normals Destination array.
 * @param vertices List of vertex coordinates.
 * @param indices List of Vertex indices.
 * @param numV Number of vertices.
 * @param numI Number of indices.
 */
void Mesh::computeNormals(GLfloat* normals, GLfloat* vertices, GLuint* indices, int numV, int numI) {
  int triangleCount[numV/3] = {0};
  // printf("%d\n", numI*3);
  for(int i=0; i<numI; i+=3) {
    int v1Index = 3*indices[i];
    int v2Index = 3*indices[i+1];
    int v3Index = 3*indices[i+2];
    GLfloat p1[3] = {vertices[v1Index], vertices[v1Index+1], vertices[v1Index+2]};
    GLfloat p2[3] = {vertices[v2Index], vertices[v2Index+1], vertices[v2Index+2]};
    GLfloat p3[3] = {vertices[v3Index], vertices[v3Index+1], vertices[v3Index+2]};
    GLfloat norm[3];
    triangleNormal(norm, p1, p2, p3);
    normals[v1Index]   += norm[0];
    normals[v1Index+1] += norm[1];
    normals[v1Index+2] += norm[2];
    normals[v2Index]   += norm[0];
    normals[v2Index+1] += norm[1];
    normals[v2Index+2] += norm[2];
    normals[v3Index]   += norm[0];
    normals[v3Index+1] += norm[1];
    normals[v3Index+2] += norm[2];
    triangleCount[indices[i]]++;
    triangleCount[indices[i+1]]++;
    triangleCount[indices[i+2]]++;
  }

  // printf("Whaaaat???\n");
  
  
  // Average and normalize normals
//   for(int i=0; i<numV; i+=3) {
//     normals[i]   /= triangleCount[indices[i/3]];
//     normals[i+1] /= triangleCount[indices[i/3 + 1]];
//     normals[i+2] /= triangleCount[indices[i/3 + 2]];
// 
//     GLfloat len = vecNorm(Vector{normals[i], normals[i+1], normals[i+2]});
//     normals[i]   /= len;
//     normals[i+1] /= len;
//     normals[i+2] /= len;
//   }
}


void Mesh::loadVertexData(GLfloat vertices[], GLfloat normals[], GLuint indices[],
                          int numV, int numI)
{
  GLuint vertexBuf;
  GLuint indexBuf;

  // Load vertex coordinates and normals
  glGenBuffers(1, &vertexBuf);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuf);
  glBufferData(GL_ARRAY_BUFFER, 2*numV*sizeof(GLfloat), NULL, GL_STATIC_DRAW);
  glBufferSubData(GL_ARRAY_BUFFER, 0, numV*sizeof(GLfloat), vertices);
  glBufferSubData(GL_ARRAY_BUFFER, numV*sizeof(GLfloat), numV*sizeof(GLfloat), normals);
  
  // Load vertex indices
  glGenBuffers(1, &indexBuf);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuf);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, numI * sizeof(GLuint), indices, GL_STATIC_DRAW);
  
  // Store data
  this->vertexBuf = vertexBuf;
  this->indexBuf = indexBuf;
  this->numTris = numI / 3;
  this->vertexSize = numV * sizeof(GLfloat);

  // Generate material uniform block buffer
  glGenBuffers(1, &materialBuf);
}


void Mesh::setMaterial(const Material& material) {
	glBindBuffer(GL_UNIFORM_BUFFER, materialBuf);
	glBufferData(GL_UNIFORM_BUFFER, sizeof(material), &material, GL_STATIC_DRAW);
}


/**
 * @brief Sends the mesh's vertex data to OpenGL for displaying.
 * @param program Shader program.
 */
void Mesh::sendVertexData(GLuint program) {
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuf);
  GLuint vPosition = glGetAttribLocation(program, "vPosition");
  glVertexAttribPointer(vPosition, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(vPosition);
  GLuint vNormal = glGetAttribLocation(program, "vNormal");
  glVertexAttribPointer(vNormal, 3, GL_FLOAT, GL_FALSE, 0, (void*)(vertexSize));
  glEnableVertexAttribArray(vNormal);
}


void Mesh::display(GLuint shaderProgram, const glm::mat4& viewPerspective) {
  sendVertexData(shaderProgram);

  // Set uniform variables
	glBindBufferBase(GL_UNIFORM_BUFFER, 2, materialBuf);
  int mvpLoc = glGetUniformLocation(shaderProgram, "modelViewPerspective");
  glUniformMatrix4fv(mvpLoc, 1, 0, glm::value_ptr(viewPerspective));

  // Draw mesh
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuf);
  glDrawElements(GL_TRIANGLES, 3*numTris, GL_UNSIGNED_INT, NULL);
}


/*******************************************************************************
 *  SOFT CUBE MESH
 ******************************************************************************/

SoftCubeMesh::SoftCubeMesh() {}

SoftCubeMesh::SoftCubeMesh(GLfloat vertices[], GLfloat normals[], GLuint indices[],
                           std::vector<GLuint> duplicateVertexIndices,
                           int numV, int numI, const Material& material)
  : Mesh(vertices, normals, indices, numV, numI, material)
{
  this->indices.assign(indices, indices + numI);
  this->duplicateVertexIndices = duplicateVertexIndices;
}

void SoftCubeMesh::bindCube(const SoftBody* cube) {
  this->cube = cube;
}


void SoftCubeMesh::update() {

  // Update vertex positions
  const std::vector<Mass*>& masses = cube->getSurfaceMasses();
  int numMasses = masses.size();
  int numV = 3 * (numMasses + duplicateVertexIndices.size());
  GLfloat vertices[numV];
  for(int i=0; i<numMasses; i++) {
    Vector pos = masses[i]->getPos();
    vertices[i*3]     = pos[0];
    vertices[i*3 + 1] = pos[1];
    vertices[i*3 + 2] = pos[2];
  }
  for(int i=0; i<duplicateVertexIndices.size(); i++) {
    Vector pos = masses[duplicateVertexIndices[i]]->getPos();
    int vIndex = 3*(i + numMasses);
    vertices[vIndex]   = pos[0];
    vertices[vIndex+1] = pos[1];
    vertices[vIndex+2] = pos[2];
  }

  // printf("testing\n");
  // printf("%d, %d\n", numV, std::max_element(indices.begin(), indices.end()));

  // Update vertex normals
  GLfloat normals[numV] = {0};
  computeNormals(normals, vertices, indices.data(), numV, indices.size());

  // printf("testington\n");

  glBindBuffer(GL_ARRAY_BUFFER, vertexBuf);
  glBufferSubData(GL_ARRAY_BUFFER, 0, vertexSize, vertices);
  glBufferSubData(GL_ARRAY_BUFFER, vertexSize, vertexSize, normals);
}


void SoftCubeMesh::display(GLuint program, const glm::mat4& viewPerspective) {
  update();
  Mesh::display(program, viewPerspective);
}
