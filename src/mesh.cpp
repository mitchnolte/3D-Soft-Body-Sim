#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "mesh.h"
#include "soft_body.h"


/*******************************************************************************
 *  MESH
 ******************************************************************************/

Mesh::Mesh() {}

/**
 * @brief  Mesh constructor.
 * 
 * @param  vertices  Vertex positions.
 * @param  normals   Vertex normals.
 * @param  indices   Vertex indices.
 * @param  numV      Number of vertices.
 * @param  numI      Number of indices.
 * @param  material  Material properties of the mesh.
 */
Mesh::Mesh(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI,
           const Material& material)
{
  loadVertexData(vertices, normals, indices, numV, numI);
  setMaterial(material);
}

/**
 * @brief  Calculates the normal vector of a triangle with the 3 given points.
 * @param  normal  Destination array.
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
 * @brief  Computes the normal vectors for a mesh. Vectors are not averaged or
 *         normalized since they are already normalized by OpenGL.
 *
 * @param  normals   Destination array. Each entry should be initialized to 0.
 * @param  vertices  Array of vertex coordinates.
 * @param  indices   Array of Vertex indices.
 * @param  numV      Number of vertices.
 * @param  numI      Number of indices.
 */
void Mesh::computeNormals(GLfloat normals[], GLfloat vertices[], GLuint indices[],
                          int numV, int numI)
{
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
  }
}


/**
 * @brief  Loads the given mesh data into OpenGL.
 * 
 * @param  vertices  Vertex positions.
 * @param  normals   Vertex normals.
 * @param  indices   Vertex indices.
 * @param  numV      Number of vertices.
 * @param  numI      Number of indices.
 */
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
 * @brief  Sends the mesh's vertex data to OpenGL for rendering.
 * @param  program  Shader program.
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


/**
 * @brief  Renders the mesh.
 * 
 * @param  shaderProgram   Shader program identifier.
 * @param  viewProjection  viewProjection matrix from camera.
 */
void Mesh::display(GLuint shaderProgram, const glm::mat4& viewProjection) {
  sendVertexData(shaderProgram);

  // Set uniform variables
	glBindBufferBase(GL_UNIFORM_BUFFER, 2, materialBuf);
  int mvpLoc = glGetUniformLocation(shaderProgram, "modelViewProjection");
  glUniformMatrix4fv(mvpLoc, 1, 0, glm::value_ptr(viewProjection));

  // Draw mesh
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuf);
  glDrawElements(GL_TRIANGLES, 3*numTris, GL_UNSIGNED_INT, NULL);
}


/*******************************************************************************
 *  SOFT BODY MESH
 ******************************************************************************/

SoftBodyMesh::SoftBodyMesh() {}

/**
 * @brief  Soft body mesh constructor.
 * 
 * @param  vertices  Vertex positions.
 * @param  normals   Vertex normals.
 * @param  indices   Vertex indices.
 * @param  numV      Number of vertices.
 * @param  numI      Number of indices.
 * @param  material  Material properties of the mesh.
 */
SoftBodyMesh::SoftBodyMesh(GLfloat vertices[], GLfloat normals[], GLuint indices[],
                           const std::vector<GLuint>& massIndices,
                           int numV, int numI, const Material& material)
  : Mesh(vertices, normals, indices, numV, numI, material)
{
  this->indices.assign(indices, indices + numI);
  this->massIndices = massIndices;
}

/**
 * @brief  Binds a soft body to the mesh. The bound soft body's mass positions
 *         are used to update the mesh each frame. The given pointer must point
 *         to the copy of the soft body stored in the simulation, otherwise it
 *         will be invalid.
 *
 * @param  body  Soft body represented by the mesh.
 */
void SoftBodyMesh::bindBody(const SoftBody* body) {
  this->body = body;
}

/**
 * @brief Updates the mesh's vertex positions and recalculates the normal
 *        vectors to match the state of the bound soft body.
 */
void SoftBodyMesh::update() {
  int numV = 3 * massIndices.size();

  // Update vertex positions
  GLfloat vertices[numV];
  const std::vector<Mass>& masses = body->getMasses();
  for(int i=0; i<massIndices.size(); i++) {
    const Vector& state = masses[massIndices[i]].getState();
    vertices[3*i]   = (float)state[0];
    vertices[3*i+1] = (float)state[1];
    vertices[3*i+2] = (float)state[2];
  }

  // Update vertex normals
  GLfloat normals[numV] = {0};
  computeNormals(normals, vertices, indices.data(), numV, indices.size());

  glBindBuffer(GL_ARRAY_BUFFER, vertexBuf);
  glBufferSubData(GL_ARRAY_BUFFER, 0, vertexSize, vertices);
  glBufferSubData(GL_ARRAY_BUFFER, vertexSize, vertexSize, normals);
}


void SoftBodyMesh::display(GLuint program, const glm::mat4& viewPerspective) {
  update();
  Mesh::display(program, viewPerspective);
}
