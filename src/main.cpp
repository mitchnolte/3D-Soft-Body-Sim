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
#include "simulation.h"

#define WIN_WIDTH  640.0
#define WIN_HEIGHT 640.0


void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
  if(height == 0) height = 1;
  glfwMakeContextCurrent(window);
  glViewport(0, 0, width, height);
}

void error_callback(int error, const char* description) {
  fprintf(stderr, "Error: %s\n", description);
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
  window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "Metaverse", NULL, NULL);
  if (!window) {
    glfwTerminate();
    exit(EXIT_FAILURE);
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

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

  // Initialize simulation
  Simulation sim;

  // Main loop
  while(!glfwWindowShouldClose(window)) {
    sim.update();
    sim.display();
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwTerminate();
  return 0;
}