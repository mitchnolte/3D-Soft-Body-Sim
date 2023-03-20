#include "mesh.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>






#include <iostream>






Mesh::Mesh() {}

Mesh::Mesh(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI,
           const Material& material)
{
  loadVertexData(vertices, normals, indices, numV, numI);
  setMaterial(material);
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

GLuint Mesh::getVertexBuf() {return vertexBuf;}

GLuint Mesh::getIndexBuf() {return indexBuf;}


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
  // int softBodyLoc = glGetUniformLocation(shaderProgram, "softBody");
  // glUniform1i(softBodyLoc, 1);

  // Draw mesh
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuf);
  glDrawElements(GL_TRIANGLES, 3*numTris, GL_UNSIGNED_INT, NULL);
}


/*******************************************************************************
 *  TRANSFORMABLE MESH
 ******************************************************************************/

TransformableMesh::TransformableMesh() {
  transformation = glm::mat4(1.0);
}

void TransformableMesh::translate(const glm::vec3& translation_vec) {
  transformation = glm::translate(transformation, translation_vec);
}

void TransformableMesh::rotate(float angle, const glm::vec3& axis) {
  transformation = glm::rotate(transformation, angle, axis);
}

void TransformableMesh::display(GLuint shaderProgram, const glm::mat4& viewPerspective) {
  sendVertexData(shaderProgram);

  // Set uniform variables
  int modelLoc = glGetUniformLocation(shaderProgram, "model");
  glUniformMatrix4fv(modelLoc, 1, 0, glm::value_ptr(transformation));
  glm::mat4 modelViewPerspective = viewPerspective * transformation;
  int mvpLoc = glGetUniformLocation(shaderProgram, "modelViewPerspective");
  glUniformMatrix4fv(mvpLoc, 1, 0, glm::value_ptr(modelViewPerspective));
  glm::mat3 normalMat = glm::transpose(glm::inverse(glm::mat3(transformation)));
  int normalMatLoc = glGetUniformLocation(shaderProgram, "normalMat");
  glUniformMatrix3fv(normalMatLoc, 1, 0, glm::value_ptr(normalMat));

  // Draw mesh
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuf);
  glDrawElements(GL_TRIANGLES, 3*numTris, GL_UNSIGNED_INT, NULL);
}
