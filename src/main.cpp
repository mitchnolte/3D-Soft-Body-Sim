#include <windows.h>
#include <GL/glew.h>
#define GLFW_DLL
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLM_FORCE_RADIANS
#include <thread>
#include <ctime>
#include <iostream>
#include "shaders.h"
#include "renderer.h"
#include "simulation.h"
#include "soft_body_factory.h"


#define WIN_WIDTH  640.0
#define WIN_HEIGHT 640.0
#define FRAME_RATE 60     // Display frames per second
#define STEP_RATE  60     // Simulation updates per second

Renderer renderer(FRAME_RATE);
Simulation sim(1.0/STEP_RATE, 4);
bool simulationRunning = true;


void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
  if(height == 0) height = 1;
  glfwMakeContextCurrent(window);
  glViewport(0, 0, width, height);
}

void error_callback(int error, const char* description) {
  std::cerr << "Error: " << description << std::endl;
}

void keyCallback(GLFWwindow* window, int key, int code, int action, int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GLFW_TRUE);

  renderer.handleKeyInput(key, action);
}

DWORD WINAPI simulationUpdateLoop(LPVOID) {
  const double UPDATE_TIME = 1000.0 / STEP_RATE;
  while(simulationRunning) {
    long startTime = std::clock();
    sim.update();
    long endTime = std::clock();
    Sleep(UPDATE_TIME - (endTime - startTime) / (double)CLOCKS_PER_SEC);
  }
  return 0;
}


int main() {
  glfwSetErrorCallback(error_callback);
  
  // Initialize glfw
  if (!glfwInit()) {
    std::cerr << "Error initializing GLFW." << std::endl;
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
    std::cerr << "Error initializing GLEW: " << glewGetErrorString(error) << std::endl;
    exit(EXIT_FAILURE);
  }

  


  // MOVE THIS TO RENDERER *************************************************************************
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glViewport(0, 0, WIN_WIDTH, WIN_HEIGHT);
  glfwSwapInterval(1);
  // ***********************************************************************************************





  // Initialize simulation
  SoftBodyFactory factory;
  SoftBody cube;
  SoftCubeMesh cubeMesh;
  std::tie(cube, cubeMesh) = factory.buildCube(Vector(3), 1, 3, 1.5, 0.1);
  sim.addBody(cube);
  cubeMesh.bindCube(&sim.getBodies()[0]);




  // STABLE PARAMETERS:
  // - factory.buildCube(Vector(3), 1, 3, 0.1, 0.008) with 4 integration steps at 60 UPS
  // - factory.buildCube(Vector(3), 1, 3, 1,   0.04)  with 4 integration steps at 60 UPS
  // - factory.buildCube(Vector(3), 1, 3, 1,5, 0.1)   with 4 integration steps at 60 UPS
  //     - Not stable with 1 integration step at 240 UPS, which I thought would be equivalent




  // Initialize renderer
  float aspectRatio = WIN_WIDTH / WIN_HEIGHT;
  GLuint program = buildProgram(buildShader(GL_VERTEX_SHADER, "vertex_shader.vs"),
                                buildShader(GL_FRAGMENT_SHADER, "fragment_shader.fs"), 0);
  renderer.setProgram(program);
  renderer.setLight({{-500, -500, 1000, 0}, {1.0, 1.0, 1.0, 1.0}});
  renderer.initializeCamera(glm::vec3(0.0, -2.0, 1.0), glm::vec3(0.0, 1.0, -0.3), aspectRatio);
  renderer.addMesh(cubeMesh);

  // Start simulation loop
  HANDLE simThread = CreateThread(NULL, 0, simulationUpdateLoop, NULL, 0, NULL);

  // Display loop
  while(!glfwWindowShouldClose(window)) {
    renderer.display();
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  
  simulationRunning = false;
  WaitForSingleObject(simThread, INFINITE);
  glfwTerminate();
  return 0;
}
