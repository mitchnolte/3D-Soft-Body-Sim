#version 420 core

in vec4 position;
in vec3 normal;
uniform vec3 camPos;

layout(std140, binding=1) uniform Light {
  vec4 lightPos;
  vec4 lightColour;
};

layout(std140, binding=2) uniform Material {
  vec4 colour;
  float reflectivity;
};


void main() {
  vec3 N = normalize(normal);
  vec3 L = normalize(lightPos.xyz - position.xyz);
  vec3 H = normalize(L + normalize(camPos - position.xyz));
  float n = 100.0 * reflectivity;
  float diffuse = dot(N, L);
  float specular;

  if(diffuse < 0.0) {
    diffuse = 0.0;
    specular = 0.0;
  }else {
    specular = pow(max(0.0, dot(N, H)), n);
  }

  gl_FragColor = min(0.3*colour + diffuse*colour*lightColour + reflectivity*lightColour*specular, vec4(1.0));
  gl_FragColor.a = colour.a;
}
