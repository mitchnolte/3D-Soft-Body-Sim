#version 330 core

in vec4 vPosition;
in vec3 vNormal;
// uniform mat4 model;
uniform mat4 modelViewPerspective;
// uniform mat3 normalMat;
uniform bool softBody;
out vec4 position;
// out vec3 normal;

void main() {
  gl_Position = modelViewPerspective * vPosition;
  // normal = normalMat * vNormal;

  // if(softBody) position = vPosition;
  // else         position = model * vPosition;
  position = vPosition;
}
