#include <Windows.h>
#include <GL/glew.h>
#define GLFW_DLL
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include "shaders.h"
#include "renderer.h"
#include "simulation.h"
#include "soft_body_factory.h"

#define WIN_WIDTH  640.0
#define WIN_HEIGHT 640.0
#define FRAME_RATE 60     // Display frames per second
#define STEP_RATE  60     // Simulation updates per second


Renderer renderer;

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
  if(height == 0) height = 1;
  glfwMakeContextCurrent(window);
  glViewport(0, 0, width, height);
}

void error_callback(int error, const char* description) {
  fprintf(stderr, "Error: %s\n", description);
}

void keyCallback(GLFWwindow* window, int key, int code, int action, int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GLFW_TRUE);

  renderer.handleKeyInput(key, action);
}


int main() {
  glfwSetErrorCallback(error_callback);
  
  // Initialize glfw
  if (!glfwInit()) {
    fprintf(stderr, "Can't initialize GLFW\n");
    exit(EXIT_FAILURE);
  }

  // Create window
  GLFWwindow* window;
  window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "Soft Body Simulation", NULL, NULL);
  if (!window) {
    glfwTerminate();
    exit(EXIT_FAILURE);
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
  glfwSetKeyCallback(window, keyCallback);

  // Initialize glew
  GLenum error = glewInit();
  if(error != GLEW_OK) {
    fprintf(stderr, "Error starting GLEW: %s\n", glewGetErrorString(error));
    exit(EXIT_FAILURE);
  }

  glEnable(GL_DEPTH_TEST);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glViewport(0, 0, WIN_WIDTH, WIN_HEIGHT);
  glfwSwapInterval(1);

  // Load shader program
  GLuint program = buildProgram(buildShader(GL_VERTEX_SHADER, "vertex_shader.vs"),
                                buildShader(GL_FRAGMENT_SHADER, "fragment_shader.fs"), 0);
  glUseProgram(program);

  // Initialize simulation
  Simulation sim(1/STEP_RATE);
  SoftBodyFactory factory;
  SoftBody cube;
  Mesh cubeMesh;
  std::tie(cube, cubeMesh) = factory.buildCube(Vector(3), 1, 165);
  sim.addBody(cube);

  // Initialize renderer
  float ratio = WIN_WIDTH / WIN_HEIGHT;
  renderer = Renderer(FRAME_RATE);
  renderer.setProgram(program);
  renderer.setLight({{-500, -500, 1000, 0}, {1.0, 1.0, 1.0, 1.0}});
  renderer.initializeCamera(glm::vec3(0.0, -5.0, 1.8), glm::vec3(0.0, 1.0, 0.0), ratio);
  renderer.bindMesh(cubeMesh, &sim.getBodies()[0]);

  // Main loop
  while(!glfwWindowShouldClose(window)) {
    // sim.update();
    renderer.display(sim);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwTerminate();
  return 0;
}
