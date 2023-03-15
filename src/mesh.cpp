#include "mesh.h"
#include <glm/gtc/type_ptr.hpp>


Mesh::Mesh() {}

GLuint Mesh::getVertexBuf() {return vertexBuf;}

GLuint Mesh::getIndexBuf() {return indexBuf;}


/**
 * @brief Loads the mesh's vertex data for displaying.
 * @param program Shader program.
 */
void Mesh::loadVertexData(GLuint program) {
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuf);
  GLuint vPosition = glGetAttribLocation(program, "vPosition");
  glVertexAttribPointer(vPosition, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(vPosition);
  GLuint vNormal = glGetAttribLocation(program, "vNormal");
  glVertexAttribPointer(vNormal, 3, GL_FLOAT, GL_FALSE, 0, (void*)(vertexSize));
  glEnableVertexAttribArray(vNormal);
}


void Mesh::display(GLuint program, const glm::mat4& viewPerspective) {
  loadVertexData(program);

  // Set uniform variables
  int mvpLoc = glGetUniformLocation(program, "modelViewPerspective");
  glUniformMatrix4fv(mvpLoc, 1, 0, glm::value_ptr(viewPerspective));

  // Draw mesh
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuf);
  glDrawElements(GL_TRIANGLES, 3*numTris, GL_UNSIGNED_INT, NULL);
}


/*******************************************************************************
 *  TRANSFORMABLE MESH
 ******************************************************************************/

void TransformableMesh::display(GLuint program, const glm::mat4& viewPerspective) {
  loadVertexData(program);

  // Set uniform variables
  int modelLoc = glGetUniformLocation(program, "model");
  glUniformMatrix4fv(modelLoc, 1, 0, glm::value_ptr(transformation));
  glm::mat4 modelViewPerspective = viewPerspective * transformation;
  int mvpLoc = glGetUniformLocation(program, "modelViewPerspective");
  glUniformMatrix4fv(mvpLoc, 1, 0, glm::value_ptr(modelViewPerspective));
  glm::mat3 normalMat = glm::transpose(glm::inverse(glm::mat3(transformation)));
  int normalMatLoc = glGetUniformLocation(program, "normalMat");
  glUniformMatrix3fv(normalMatLoc, 1, 0, glm::value_ptr(normalMat));

  // Draw mesh
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuf);
  glDrawElements(GL_TRIANGLES, 3*numTris, GL_UNSIGNED_INT, NULL);
}
