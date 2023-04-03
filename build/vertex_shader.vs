#version 330 core

in vec4 vPosition;
in vec3 vNormal;
uniform mat4 modelViewProjection;
out vec4 position;
out vec3 normal;

void main() {
  gl_Position = modelViewProjection * vPosition;
  position = vPosition;
  normal = vNormal;
}
