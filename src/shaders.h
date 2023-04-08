#ifndef SHADERS_H
#define SHADERS_H

#include <GL/glew.h>


GLuint buildShaderProgram() {

  const char* vertexShader =
    "#version 330 core\n"

    "in vec4 vPosition;\n"
    "in vec3 vNormal;\n"
    "uniform mat4 modelViewProjection;\n"
    "out vec4 position;\n"
    "out vec3 normal;\n"

    "void main() {\n"
    "  gl_Position = modelViewProjection * vPosition;\n"
    "  position = vPosition;\n"
    "  normal = vNormal;\n"
    "}\n\0";


  const char* fragmentShader =
    "#version 420 core\n"

    "in vec4 position;\n"
    "in vec3 normal;\n"
    "uniform vec3 camPos;\n"

    "layout(std140, binding=1) uniform Light {\n"
    "  vec4 lightPos;\n"
    "  vec4 lightColour;\n"
    "};\n"

    "layout(std140, binding=2) uniform Material {\n"
    "  vec4 colour;\n"
    "  float reflectivity;\n"
    "};\n"


    "void main() {\n"
    "  vec3 N = normalize(normal);\n"
    "  vec3 L = normalize(lightPos.xyz - position.xyz);\n"
    "  vec3 H = normalize(L + normalize(camPos - position.xyz));\n"
    "  float n = 100.0 * reflectivity;\n"
    "  float diffuse = dot(N, L);\n"
    "  float specular;\n"

    "  if(diffuse < 0.0) {\n"
    "    diffuse = 0.0;\n"
    "    specular = 0.0;\n"
    "  }else {\n"
    "    specular = pow(max(0.0, dot(N, H)), n);\n"
    "  }\n"

    "  gl_FragColor = min(0.3*colour + diffuse*colour*lightColour +\n"
    "                    reflectivity*lightColour*specular, vec4(1.0));\n"
    "  gl_FragColor.a = colour.a;\n"
    "}\n\0";


  GLuint vs = glCreateShader(GL_VERTEX_SHADER);
  GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(vs, 1, &vertexShader, NULL);
  glShaderSource(fs, 1, &fragmentShader, NULL);
  glCompileShader(vs);
  glCompileShader(fs);

  GLuint program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);
  glDeleteShader(vs);
  glDeleteShader(fs);

  return program;
}

#endif
