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
#include "rigid_body.h"


#define WIN_WIDTH  640.0  // Initial window width
#define WIN_HEIGHT 640.0  // Initial window height
#define FOV        1.57   // Field of view (90 degrees)
#define FRAME_RATE 60     // Display frames per second
#define STEP_RATE  60     // Simulation updates per second
#define RK4_ITERS  4      // Number of RK4 iterations per update

Renderer renderer(FRAME_RATE);
Simulation sim(1.0/STEP_RATE, RK4_ITERS);
bool simulationRunning = true;


/**
 * @brief GLFW frame buffer size callback function.
 */
void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
  if(height == 0) height = 1;
  glfwMakeContextCurrent(window);
  renderer.setViewport(width, height);
}

/**
 * @brief GLFW error callback function.
 */
void errorCallback(int error, const char* description) {
  std::cerr << "Error: " << description << std::endl;
}

/**
 * @brief GLFW key callback function.
 */
void keyCallback(GLFWwindow* window, int key, int code, int action, int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GLFW_TRUE);

  renderer.handleKeyInput(key, action);
}

/**
 * @brief Simulation update thread function.
 */
DWORD WINAPI simulationUpdateLoop(LPVOID args) {
  const double UPDATE_DURATION = 1000.0 / STEP_RATE;
  while(simulationRunning) {
    long startTime = std::clock();
    sim.update();
    long endTime = std::clock();
    Sleep(UPDATE_DURATION - (endTime - startTime) / (double)CLOCKS_PER_SEC);
  }
  return 0;
}


int main() {
  glfwSetErrorCallback(errorCallback);
  
  // Initialize glfw
  if(!glfwInit()) {
    std::cerr << "Error initializing GLFW." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Create window
  GLFWwindow* window;
  window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "Soft Body Simulation", NULL, NULL);
  if(!window) {
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

  // Initialize simulation
  SoftBodyFactory factory;
  SoftCube cube;
  SoftCubeMesh cubeMesh;
  std::tie(cube, cubeMesh) = factory.buildCube(Vector(3), 1, 3, 1.5, 0.1);

  // STABLE PARAMETERS:
  // - factory.buildCube(Vector(3), 1, 3, 0.1, 0.008) with 4 integration steps at 60 UPS
  // - factory.buildCube(Vector(3), 1, 3, 1,   0.04)  with 4 integration steps at 60 UPS
  // - factory.buildCube(Vector(3), 1, 3, 1,5, 0.1)   with 4 integration steps at 60 UPS
  //     - Not stable with 1 integration step at 240 UPS, which I thought would be equivalent

  sim.addBody(cube);
  cubeMesh.bindCube(sim.getSoftBodies()[0]);

  RigidRectPrism rect(Vector{0, 0, -3}, 4, 4, 2, 0.52, Vector{0.2, 0.8, 0.5});
  sim.addBody(rect);




  // Initialize renderer
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glViewport(0, 0, WIN_WIDTH, WIN_HEIGHT);
  glfwSwapInterval(1);
  GLuint program = buildProgram(buildShader(GL_VERTEX_SHADER, "vertex_shader.vs"),
                                buildShader(GL_FRAGMENT_SHADER, "fragment_shader.fs"), 0);
  renderer.setProgram(program);
  renderer.setLight({{-500, -500, 1000, 0}, {1.0, 1.0, 1.0, 1.0}});
  renderer.initializeCamera(glm::vec3(0.0, -2.0, 1.0), glm::vec3(0.0, 1.0, -0.3), FOV, 
                            WIN_WIDTH/WIN_HEIGHT);
  renderer.addMesh(cubeMesh);
  renderer.addMesh(rect.buildMesh());

  // Simulation loop
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
