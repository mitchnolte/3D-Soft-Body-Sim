#include <windows.h>
#include <GL/glew.h>
#define GLFW_DLL
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <thread>
#include <ctime>
#include <iostream>
#include "shaders.h"
#include "renderer.h"
#include "mesh.h"
#include "simulation.h"
#include "soft_body.h"
#include "soft_cube.h"
#include "rigid_body.h"

#define FULLSCREEN 0
#if FULLSCREEN == 1
  #define WIN_WIDTH  1920.0
  #define WIN_HEIGHT 1080.0
#else
  #define WIN_WIDTH  840
  #define WIN_HEIGHT 840
#endif
#define FOV        1.57   // Field of view (in radians; 90 degrees)
#define FRAME_RATE 60     // Display frames per second
#define STEP_RATE  60     // Simulation updates per second
#define RK4_ITERS  4      // Number of RK4 iterations per update

Renderer renderer(FRAME_RATE);
Simulation sim(1.0/STEP_RATE, RK4_ITERS);
bool simulationRunning = true;            // Used to stop simulation thread


/**
 * @brief GLFW error callback function.
 */
void errorCallback(int error, const char* description) {
  std::cerr << "Error: " << description << std::endl;
}

/**
 * @brief GLFW frame buffer size callback function.
 */
void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
  if(height == 0) height = 1;
  glfwMakeContextCurrent(window);
  renderer.setViewport(width, height);
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
 * @brief Creates the objects for the simulation, passes their meshes to the
 *        renderer, and sets the camera, lighting, and background color
 *        parameters of the renderer.
 */
void buildSimulation() {
  float     backgroundColor[3] = { 0.0,  0.0,  0.0};
  float     lightPosition[3]   = {-500, -500, 1000};
  float     lightColor[3]      = { 1.0,  1.0,  1.0};
  glm::vec3 camPosition        = { 2.5, -6.0,  1.0};    // Position for viewing collision
//glm::vec3 camPosition        = { 2.0, -3.0,  1.0};    // Position for viewing collision
//glm::vec3 camPosition        = { 4.0, -6.0, -1.0};    // Position for viewing collision
//glm::vec3 camPosition        = { 0.0, -2.0,  1.0};    // Position for viewing cube alone
  glm::vec3 camDirection       = { 0.2,  1.0, -0.6};
//glm::vec3 camDirection       = {-4.0,  6.0, -3.0};

  Vector   cubePos   = {0.0, 0.0, 0.0};      // Position
  double   cubeSize  = 1.0;                  // Side length
  int      cubeCPA   = 2;                    // Cells per axis (determines number of point masses)
  double   k         = 3000;                  // Spring coefficient
//double   k         = 5000;                  // Spring coefficient
  double   c         = 1;                    // Damping coefficient
  Material cubeMat   = {{1, 0, 0, 1}, 1};    // Color and reflectivity

//Vector   rectPos   = {0.0, 0.0, -1.7};
//Vector   rectPos   = {-2.3, 0.0, -1.7};
//Vector   rectPos   = {0.0, 0.0, -2.0};
  Vector   rectPos   = {1.0, 0.0, -4.0};
//Vector   rectPos   = {0.0, 0.0, -7.0};
//Vector   rectPos   = {0.0, 0.0, -10.0};
  double   rectLenX  = 4;
  double   rectLenY  = rectLenX;
  double   rectLenZ  = 2;
//double   rectAngle = 0.0;
  double   rectAngle = 0.1;
//double   rectAngle = 0.52;
//Vector   rectAxis  = {0.2, 0.8, 0.5};
  Vector   rectAxis  = {0.0, 1.0, 0.0};

  // Soft body
  SoftCube cube(1);
  SoftBodyMesh cubeMesh = cube.buildStructure(cubePos, cubeSize, cubeCPA, k, c, cubeMat);
  sim.addBody(cube);
  cubeMesh.bindBody(sim.getSoftBodies()[0]);

  // Rigid bodies
  RigidRectPrism rect(rectPos, rectLenX, rectLenY, rectLenZ, rectAngle, rectAxis);
  sim.addBody(rect);

  // Visualization
  glClearColor(backgroundColor[0], backgroundColor[1], backgroundColor[2], 1.0);
  renderer.setLight({{lightPosition[0], lightPosition[1], lightPosition[2], 0},
                     {   lightColor[0],    lightColor[1],    lightColor[2], 0}});
  renderer.initializeCamera(camPosition, camDirection, FOV, WIN_WIDTH/WIN_HEIGHT);
  renderer.addMesh(cubeMesh);
  renderer.addMesh(rect.buildMesh());
}


/**
 * @brief Simulation thread function. Updates simulation at the rate defined by
 *        STEP_RATE until simulationRunning is set to false.
 */
DWORD WINAPI runSimulation(LPVOID args) {
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
  
  // Initialize glfw
  glfwSetErrorCallback(errorCallback);
  if(!glfwInit()) {
    std::cerr << "Error initializing GLFW." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Create window
  GLFWwindow* window;
  GLFWmonitor* monitor = NULL;

#if FULLSCREEN == 1
  monitor = glfwGetPrimaryMonitor();
#endif

  window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "Soft Body Simulation", monitor, NULL);
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

  // Initialize renderer
  glEnable(GL_DEPTH_TEST);
  glViewport(0, 0, WIN_WIDTH, WIN_HEIGHT);
  glfwSwapInterval(1);
  GLuint program = buildProgram(buildShader(GL_VERTEX_SHADER, "vertex_shader.vs"),
                                buildShader(GL_FRAGMENT_SHADER, "fragment_shader.fs"), 0);
  renderer.setProgram(program);

  // Initialize simulation
  buildSimulation();
  HANDLE simThread = CreateThread(NULL, 0, runSimulation, NULL, 0, NULL);

  // Display loop
  while(!glfwWindowShouldClose(window)) {
    renderer.display();
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  
  glfwTerminate();
  simulationRunning = false;
  WaitForSingleObject(simThread, INFINITE);
  return 0;
}
