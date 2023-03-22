#version 330 core

in vec4 vPosition;
in vec3 vNormal;
uniform mat4 modelViewPerspective;
out vec4 position;
out vec3 normal;

void main() {
  gl_Position = modelViewPerspective * vPosition;
  position = vPosition;
  normal = vNormal;
}
