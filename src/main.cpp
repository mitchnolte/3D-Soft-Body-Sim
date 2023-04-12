#include <windows.h>
#include <GL/glew.h>
#define GLFW_DLL
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <chrono>
#include <iostream>
#include "shaders.h"
#include "renderer.h"
#include "mesh.h"
#include "simulation.h"
#include "soft_body.h"
#include "soft_cube.h"
#include "rigid_body.h"


#define WIN_WIDTH  840    // Windowed mode width
#define WIN_HEIGHT 840    // Windowed mode height
#define FOV        1.57   // Field of view (in radians; 90 degrees)
#define FRAME_RATE 60     // Display frames per second
#define STEP_RATE  60     // Simulation updates per second
#define RK4_ITERS  4      // Number of RK4 iterations per update

Renderer   renderer(FRAME_RATE);
Simulation sim(1.0/STEP_RATE, RK4_ITERS);
bool       simulationRunning;             // Used to stop simulation thread
bool       displayStepRate;               // Whether the simulation update rate should be printed
int        winWidth;                      // Current window width
int        winHeight;                     // Current window height


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
  winWidth = width;
  winHeight = height;
}

/**
 * @brief GLFW key callback function.
 */
void keyCallback(GLFWwindow* window, int key, int code, int action, int mods) {
  if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  else if(key == GLFW_KEY_F && action == GLFW_PRESS) {
    displayStepRate = !displayStepRate;
    if(displayStepRate)
      std::cout << "\nUpdates per second:" << std::endl;
  }

  renderer.handleKeyInput(key, action);
}

/**
 * @brief GLFW mouse cursor position callback.
 */
void mouseCallback(GLFWwindow* window, double x, double y) {
  glfwSetCursorPos(window, 0, 0);
  renderer.handleMouseInput(x, y);
}


/**
 * @brief  Creates the objects for the simulation, passes their meshes to the
 *         renderer, and sets the camera, lighting, and background color
 *         parameters of the renderer.
 *
 * @param  aspectRatio  Window aspect ratio used to initialize camera.
 */
void buildSimulation(float aspectRatio) {
  float     backgroundColor[3] = { 0.0,  0.0,  0.0};
  float     lightColor[3]      = { 1.0,  1.0,  1.0};
  float     lightPosition[3]   = {-300, -700,  500};
  glm::vec3 camPosition        = { 2.0, -8.0, -2.0};
  glm::vec3 camDirection       = { 0.0,  1.0, -0.3};

  Vector   cubePos  = {0.0, 0.0, 0.0};      // Position
  double   cubeSize = 1.0;                  // Side length
  int      cubeCPA  = 2;                    // Cells per axis (determines number of point masses)
  double   k        = 1990;                 // Spring coefficient
  double   c        = 0.8;                  // Damping coefficient
  Material cubeMat  = {{1, 0, 0, 1}, 1};    // Color and reflectivity

  Vector   rect1Pos   = cubePos  + Vector{ 1.0,  0.0, -4.0};
  Vector   rect2Pos   = rect1Pos + Vector{ 6.0,  0.0, -1.8};
  Vector   rect3Pos   = rect2Pos + Vector{-2.1,  0.0, -2.5};
  Vector   rect4Pos   = rect3Pos + Vector{-6.5,  0.0, -6.0};
  Vector   rect5Pos   = rect4Pos + Vector{ 7.0, -1.0, -3.0};
  Vector   rect6Pos   = rect5Pos + Vector{ 0.0, -5.0, -0.0};
  Vector   rect7Pos   = rect6Pos + Vector{-7.0,  2.0, -2.0};
  Vector   rect8Pos   = rect7Pos + Vector{ 5.0,  1.5, -7.0};
  Vector   rect9Pos   = rect8Pos + Vector{ 5.0,  0.0,  0.0};
  double   rectLenX   = 5;
  double   rectLenY   = rectLenX;
  double   rectLenZ   = 1;
  double   rect1Angle =  0.2;
  double   rect2Angle =  1.57;
  double   rect3Angle = -0.2;
  double   rect4Angle =  0.6;
  double   rect5Angle = -0.8;
  double   rect6Angle = -0.8;
  double   rect7Angle =  1.57;
  Vector   rect5Axis  = {-1.0, 0.8, 0.0};
  Vector   rect6Axis  = { 1.0, 0.8, 0.0};
  Vector   yAxis      = { 0.0, 1.0, 0.0};
  Vector   xAxis      = { 1.0, 0.0, 0.0};
  Material rectMat    = {{0.3, 0.3, 0.3, 1.0}, 0.2};

  // Soft body
  SoftCube cube;
  SoftBodyMesh cubeMesh = cube.buildStructure(cubePos, cubeSize, cubeCPA, k, c, cubeMat);
  sim.addBody(cube);
  cubeMesh.bindBody(sim.getSoftBodies()[0]);

  // Rigid bodies
  RigidRectPrism rect1(rect1Pos, rectLenX, rectLenY, rectLenZ, rect1Angle, yAxis);
  RigidRectPrism rect2(rect2Pos, rectLenX, rectLenY, rectLenZ, rect2Angle, yAxis);
  RigidRectPrism rect3(rect3Pos, rectLenX, rectLenY, rectLenZ, rect3Angle, yAxis);
  RigidRectPrism rect4(rect4Pos, rectLenX, rectLenY, rectLenZ, rect4Angle, yAxis);
  RigidRectPrism rect5(rect5Pos, rectLenX, rectLenY, rectLenZ, rect5Angle, rect5Axis);
  RigidRectPrism rect6(rect6Pos, rectLenX, rectLenY, rectLenZ, rect6Angle, rect6Axis);
  RigidRectPrism rect7(rect7Pos, rectLenX, rectLenY, rectLenZ, rect7Angle, yAxis);
  RigidRectPrism rect8(rect8Pos, rectLenX, rectLenY, rectLenZ);
  RigidRectPrism rect9(rect9Pos, rectLenX, rectLenY, rectLenZ);
  sim.addBody(rect1);
  sim.addBody(rect2);
  sim.addBody(rect3);
  sim.addBody(rect4);
  sim.addBody(rect5);
  sim.addBody(rect6);
  sim.addBody(rect7);
  sim.addBody(rect8);
  sim.addBody(rect9);

  // Visualization
  glClearColor(backgroundColor[0], backgroundColor[1], backgroundColor[2], 1.0);
  renderer.setLight({{lightPosition[0], lightPosition[1], lightPosition[2], 0},
                     {lightColor[0], lightColor[1], lightColor[2], 0}});
  renderer.initializeCamera(camPosition, camDirection, FOV, aspectRatio);
  renderer.addMesh(cubeMesh);
  renderer.addMesh(rect1.buildMesh(rectMat));
  renderer.addMesh(rect2.buildMesh(rectMat));
  renderer.addMesh(rect3.buildMesh(rectMat));
  renderer.addMesh(rect4.buildMesh(rectMat));
  renderer.addMesh(rect5.buildMesh(rectMat));
  renderer.addMesh(rect6.buildMesh(rectMat));
  renderer.addMesh(rect7.buildMesh(rectMat));
  renderer.addMesh(rect8.buildMesh(rectMat));
  renderer.addMesh(rect9.buildMesh(rectMat));
}


/**
 * @brief Simulation thread entry point. Updates simulation at the rate defined
 *        by STEP_RATE until simulationRunning is set to false and prints the
 *        current number of updates per second if displayStepRate is true.
 */
DWORD WINAPI runSimulation(LPVOID args) {
  using namespace std::chrono;
  using namespace std::chrono_literals;
  using SleepTime = duration<double, std::ratio<1, (int)1e7>>;

  duration<double, std::nano> updateDuration = 1s / (double)STEP_RATE - 300us;
  steady_clock::time_point    startTime;
  steady_clock::time_point    endTime;
  steady_clock::time_point    lastPrintTime  = steady_clock::now();
  float stepRate = STEP_RATE;
  int   updates  = 0;

  HANDLE timer = CreateWaitableTimer(NULL, TRUE, NULL);
  LARGE_INTEGER sleepTime;

  timeBeginPeriod(1);
  simulationRunning = true;
  while(simulationRunning) {

    // Advance simulation forward
    startTime = steady_clock::now();
    sim.update();
    updates++;
    endTime = steady_clock::now();

    // Display step rate
    if((endTime - lastPrintTime) >= 1s) {
      stepRate = 0.2*stepRate + 0.8*updates;
      lastPrintTime = steady_clock::now();
      updates = 0;
      if(displayStepRate)
        printf("%.1f\n", stepRate);
    }

    // Wait for next update time
    sleepTime.QuadPart = -duration_cast<SleepTime>(updateDuration - (endTime - startTime)).count();
    SetWaitableTimer(timer, &sleepTime, 0, NULL, NULL, FALSE);
    WaitForSingleObject(timer, INFINITE);
  }

  timeEndPeriod(1);
  CloseHandle(timer);
  return 0;
}


int main(int argc, char* argv[]) {
  
  // Initialize glfw
  glfwSetErrorCallback(errorCallback);
  if(!glfwInit()) {
    std::cerr << "Error initializing GLFW." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Command line flags
  GLFWmonitor* monitor = NULL;
  winWidth        = WIN_WIDTH;
  winHeight       = WIN_HEIGHT;
  displayStepRate = false;
  for(int i=1; i<argc; i++) {

    // -f for full screen mode
    if(strcmp(argv[i], "-f") == 0) {
      monitor = glfwGetPrimaryMonitor();
      const GLFWvidmode* mode = glfwGetVideoMode(monitor);
      winWidth = mode->width;
      winHeight = mode->height;
    }

    // -ups to start with updates-per-second counter enabled
    else if(strcmp(argv[i], "-ups") == 0)
      displayStepRate = true;
  }

  // Create window
  GLFWwindow* window;
  window = glfwCreateWindow(winWidth, winHeight, "Soft Body Simulation", monitor, NULL);
  if(!window) {
    glfwTerminate();
    exit(EXIT_FAILURE);
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

  // Initialize inputs
  glfwSetKeyCallback(window, keyCallback);
  glfwSetCursorPosCallback(window, mouseCallback);
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  glfwSetCursorPos(window, 0, 0);

  // Initialize glew
  GLenum error = glewInit();
  if(error != GLEW_OK) {
    std::cerr << "Error initializing GLEW: " << glewGetErrorString(error) << std::endl;
    exit(EXIT_FAILURE);
  }

  // Initialize renderer
  glEnable(GL_DEPTH_TEST);
  glViewport(0, 0, winWidth, winHeight);
  glfwSwapInterval(1);
  renderer.setProgram(buildShaderProgram());

  // Initialize simulation
  buildSimulation((float)winWidth/winHeight);
  HANDLE simThread = CreateThread(NULL, 0, runSimulation, NULL, 0, NULL);

  std::cout << "\n___________________________________________\n\n"
            <<   "                 CONTROLS:\n\n"
            <<   "  CAMERA CONTROL\n"
            <<   "    W          - Move forward\n"
            <<   "    A          - Move left\n"
            <<   "    S          - Move back\n"
            <<   "    D          - Move right\n"
            <<   "    SPACE      - Move up\n"
            <<   "    LEFT SHIFT - Move down\n"
            <<   "    MOUSE      - Rotate camera\n\n"
            <<   "  GENERAL\n"
            <<   "    ESC        - Close program\n"
            <<   "    F          - Toggle step rate counter\n\n"
            <<   "___________________________________________\n" << std::endl;

  if(displayStepRate)
      std::cout << "\nUpdates per second:" << std::endl;

  // Display loop
  while(!glfwWindowShouldClose(window)) {
    renderer.display();
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  
  simulationRunning = false;
  glfwTerminate();
  WaitForSingleObject(simThread, INFINITE);
  return 0;
}
