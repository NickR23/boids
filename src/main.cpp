#include <vector>
#include <random>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <raylib.h>

#define NS_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION
#include <Metal/Metal.hpp>
#include <Foundation/Foundation.hpp>
#include <QuartzCore/QuartzCore.hpp>

struct Boid {
  float position[2]; // x, y
  float velocity[2]; // vx, vy

  float seperationRange;
  float avoidFactor;
  float visualRange;
  float alignmentFactor;
  float gatheringFactor;
  float turnFactor;

  float maxSpeed;
  float minSpeed;

  float margins[4]; // {Left, Right, Bottom Top}
};

struct WorldParams {
  uint32_t numBoids;
  float xBound;
  float yBound;
};

class World {
  // TODO access qualifiers
  public:
  std::vector<Boid> boids;
  int xBound, yBound;
  float scale = 1.0f;

  MTL::Device* device;
  MTL::CommandQueue* commandQueue;
  MTL::ComputePipelineState* pipelineState;
  MTL::Buffer* boidBuffer;
  MTL::Buffer* paramsBuffer;

  World() : device(nullptr), commandQueue(nullptr), pipelineState(nullptr),
          boidBuffer(nullptr), paramsBuffer(nullptr) {};
  ~World() {
    cleanup();
  }

  void cleanup() {
    if (boidBuffer) boidBuffer->release();
    if (paramsBuffer) paramsBuffer->release();
    if (pipelineState) pipelineState->release();
    if (commandQueue) commandQueue->release();
    if (device) device->release();
  }
};

struct Options {
  int numRoids;

  int maxSpeed;
  int minSpeed; 

  float minSepRange;
  float maxSepRange;

  float minAvoidFactor;
  float maxAvoidFactor;

  float minVisualRange;
  float maxVisualRange;

  float minAlignmentFactor;
  float maxAlignmentFactor;

  float minGatheringFactor;
  float maxGatheringFactor;

  float minTurnFactor;
  float maxTurnFactor;

  float minMaxSpeed;
  float maxMaxSpeed;

  std::pair<int, int> minXMargin; // {minLeftMargin, minRightMargin}
  std::pair<int, int> maxXMargin; // {maxLeftMargin, maxRightMargin}
  std::pair<int, int> minYMargin; // {minBottomMargin, minTopMargin}
  std::pair<int, int> maxYMargin; // {maxBottomMargin, maxTopMargin}
};

float getRandom(float min, float max) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(min, max);
  return dis(gen);
}

bool initMetal(World& world) {
  world.device = MTL::CreateSystemDefaultDevice();
  if (!world.device) {
    std::cerr << "Failed to create Metal device." << std::endl;
    return false;
  }

  world.commandQueue = world.device->newCommandQueue();

  NS::Error* error = nullptr;
  MTL::Library* library = world.device->newDefaultLibrary();
  if (!library) {
    std::cerr << "Failed to create Metal library." << std::endl;
    return false;
  }

  MTL::Function* kernelFunction = library->newFunction (
      NS::String::string("updateBoids",
        NS::UTF8StringEncoding));

  world.pipelineState = world.device->newComputePipelineState(kernelFunction, &error);
  if (!world.pipelineState) {
    std::cerr << "Failed to create compute pipeline." << std::endl;
    return false;
  }

  library->release();
  kernelFunction->release();
  return true;
}

void setupMetalBuffers(World& world) {
  size_t numBoids = world.boids.size();

  world.boidBuffer = world.device->newBuffer(
      sizeof(Boid) * numBoids,
      MTL::ResourceStorageModeShared);

  world.paramsBuffer = world.device->newBuffer(
      sizeof(WorldParams),
      MTL::ResourceStorageModeShared);

  WorldParams* params = (WorldParams*)world.paramsBuffer->contents();
  params->numBoids = static_cast<uint32_t>(numBoids);
  params->xBound = world.xBound;
  params->yBound = world.yBound;
}

void copyBoidsToGPU(World& world) {
  Boid* gpuBoids = (Boid*)world.boidBuffer->contents();
  for (size_t i = 0; i < world.boids.size(); i++) {
    const Boid& boid = world.boids[i];
    Boid& gpuBoid = gpuBoids[i];
    
    gpuBoid.position[0] = boid.position[0];
    gpuBoid.position[1] = boid.position[1];

    gpuBoid.velocity[0] = boid.velocity[0];
    gpuBoid.velocity[1] = boid.velocity[1];

    gpuBoid.seperationRange = boid.seperationRange;
    gpuBoid.avoidFactor = boid.avoidFactor;
    gpuBoid.visualRange = boid.visualRange;
    gpuBoid.alignmentFactor = boid.alignmentFactor;
    gpuBoid.gatheringFactor = boid.gatheringFactor;
    gpuBoid.turnFactor = boid.turnFactor;
    gpuBoid.maxSpeed = boid.maxSpeed;
    gpuBoid.minSpeed = boid.minSpeed;

    gpuBoid.margins[0] = boid.margins[0];
    gpuBoid.margins[1] = boid.margins[1];
    gpuBoid.margins[2] = boid.margins[2];
    gpuBoid.margins[3] = boid.margins[3];
  }
}

// TODO remove the need for this. We should have direct
// access to the gpu's buffers.
void copyBoidsFromGPU(World& world) {
  Boid* gpuBoids = (Boid*)world.boidBuffer->contents();
  for (size_t i = 0; i < world.boids.size(); i++) {
    Boid& boid = world.boids[i];
    const Boid& gpuBoid = gpuBoids[i];

    boid.position[0] = gpuBoid.position[0];
    boid.position[1] = gpuBoid.position[1];

    boid.velocity[0] = gpuBoid.velocity[0];
    boid.velocity[1] = gpuBoid.velocity[1];
  }
}

void processRoidsGPU(World& world) {

  MTL::CommandBuffer* commandBuffer = world.commandQueue->commandBuffer();
  MTL::ComputeCommandEncoder* encoder = commandBuffer->computeCommandEncoder();

  encoder->setComputePipelineState(world.pipelineState);
  encoder->setBuffer(world.boidBuffer, 0, 0);
  encoder->setBuffer(world.paramsBuffer, 0, 1);

  NS::UInteger numBoids = world.boids.size();
  NS::UInteger threadGroupSize = world.pipelineState->maxTotalThreadsPerThreadgroup();
  if (threadGroupSize > numBoids) threadGroupSize = numBoids;

  MTL::Size threadsPerGroup = MTL::Size(threadGroupSize, 1, 1);
  MTL::Size numThreadsGroups = MTL::Size((numBoids + threadGroupSize - 1) / threadGroupSize, 1, 1);

  encoder->dispatchThreadgroups(numThreadsGroups, threadsPerGroup);
  encoder->endEncoding();

  commandBuffer->commit();
  commandBuffer->waitUntilCompleted();

  //copyBoidsFromGPU(world);
}

void drawRoids(const World& world) {
  BeginDrawing();
  ClearBackground(RAYWHITE);
  Boid* gpuBoids = (Boid*)world.boidBuffer->contents();
  for (size_t i = 0; i < world.boids.size(); i++) {
    DrawCircle(gpuBoids[i].position[0], gpuBoids[i].position[1], 5 * world.scale, BLACK);
  }

  DrawFPS(10, 10);
  DrawText(TextFormat("Boids: %d", (int)world.boids.size()), 10, 35, 20, GREEN);

  EndDrawing();
}

void mainLoop(World& world) {
  InitWindow(world.xBound, world.yBound, "Boids");
  copyBoidsToGPU(world);
  while (!WindowShouldClose()) {
    processRoidsGPU(world);
    drawRoids(world);
  }

  CloseWindow();
}

void run(const Options& options) {
 InitWindow(10,10,"Setting Up Boids");
 SetWindowState(FLAG_WINDOW_RESIZABLE);
 SetTargetFPS(120);

 World world;
 int maxWindowWidth = GetMonitorWidth(0) - 100; 
 int maxWindowHeight = GetMonitorHeight(0) - 150;
 CloseWindow(); // Hacky way to get monitor size.

 world.xBound = std::min(1800, maxWindowWidth);
 world.yBound = std::min(1800, maxWindowHeight);
 world.scale = 0.5f;
 if (world.xBound < 1800 || world.yBound < 1800) {
   world.scale = std::min((float)world.xBound / 1800.0f,
       (float)world.yBound / 1800.0f) * 0.5f;
 }

 if (!initMetal(world)) {
   std::cerr << "Failed to init Metal" << std::endl;
   return;
 }

 for (int i = 0; i < options.numRoids; i++) {
   // Random position across screen
   float x = getRandom(100, 1700);
   float y = getRandom(100, 1700);

   // Random velocity with proper direction and speed
   float angle = getRandom(0, 6.28318530718); // 0 to 2π radians
   float speed = getRandom(options.minSpeed, options.maxSpeed);
   float vx = speed * std::cos(angle);
   float vy = speed * std::sin(angle);

   Boid boid{
     {x, y},     // Position
     {vx, vy},   // Velocity components
     15,        // separationRange
     0.001,    // avoidFactor
     40,       // visualRange
     0.05,      // alignmentFactor
     0.005,     // gatheringFactor
     0.3,      // turnFactor
     3,        // maxSpeed
     1,        // minSpeed
     {100, 100, 100, 100}
   };

   world.boids.push_back(boid);
 }

 setupMetalBuffers(world);

 mainLoop(world);
}

int main() {
  Options options;
  options.numRoids = 9999;
  options.maxSpeed = 6;
  options.minSpeed = 4;
  options.maxTurnFactor = 0.8;
  options.minTurnFactor = 0.2;
  options.minVisualRange = 40;
  options.maxVisualRange = 80;

  run(options);
  return 0;
}
