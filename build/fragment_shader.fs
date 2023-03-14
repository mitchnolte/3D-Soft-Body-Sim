#version 330 core

in vec4 position;
in vec3 normal;
uniform vec4 colour;
uniform float reflectivity;
uniform vec3 lightPos;
uniform vec3 camPos;

void main() {
  vec4 lightColour = vec4(1.0, 1.0, 1.0, 1.0);
  vec3 N;
  vec3 L;
  vec3 H;
  float diffuse;
  float specular;
  float n = 100.0 * reflectivity;

  N = normalize(normal);
  L = normalize(lightPos - position.xyz);
  H = normalize(L + normalize(camPos - position.xyz));
  diffuse = dot(N, L);
  if(diffuse < 0.0) {
    diffuse = 0.0;
    specular = 0.0;
  } else {
    specular = pow(max(0.0, dot(N, H)), n);
  }

  gl_FragColor = min(0.3*colour + diffuse*colour*lightColour + reflectivity*lightColour*specular, vec4(1.0));
  gl_FragColor.a = colour.a;
}
