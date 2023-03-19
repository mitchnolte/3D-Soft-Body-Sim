#include "mesh.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


Mesh::Mesh() {}

Mesh::Mesh(GLfloat vertices[], GLfloat normals[], GLuint indices[], int numV, int numI) {
  loadVertexData(vertices, normals, indices, numV, numI);
}

void Mesh::loadVertexData(GLfloat vertices[], GLfloat normals[], GLuint indices[],
                          int numV, int numI)
{
  GLuint vertexBuf;
  GLuint indexBuf;

  // Load vertex coordinate data
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
  int mvpLoc = glGetUniformLocation(shaderProgram, "modelViewPerspective");
  glUniformMatrix4fv(mvpLoc, 1, 0, glm::value_ptr(viewPerspective));

  int softBodyLoc = glGetUniformLocation(shaderProgram, "softBody");
  glUniform1i(softBodyLoc, 1);

  int colourLoc = glGetUniformLocation(shaderProgram, "colour");
  glUniform4fv(colourLoc, 1, glm::value_ptr(glm::vec4(1.0, 0.0, 0.0, 1.0)));

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
