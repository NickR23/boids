#include <vector>
#include <random>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <raylib.h>

struct Roid {
  float x;
  float y;
  float vx; // velocity
  float vy; // velocity

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

struct World {
  std::vector<Roid> boids;
  int xBound;
  int yBound;
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

void drawRoids(const World& world) {
  BeginDrawing();
  ClearBackground(RAYWHITE);
  for (const Roid& boid : world.boids) {
    DrawCircle(boid.x, boid.y, 5, BLACK);
  }

  EndDrawing();
}

float getDistance(const Roid& boid, const Roid& other) {
  float dx = boid.x - other.x;
  float dy = boid.y - other.y;
  return std::sqrt(dx * dx + dy * dy);
}

// Reacts to other boids with seperationRange. Returns the new velocity vector after seperation.
std::pair<float, float> seperate(const Roid& boid, const std::vector<Roid>& flock) {
  float close_dx = 0;
  float close_dy = 0;
  for (const Roid& other : flock) {
    if (&boid == &other) continue;
    if (getDistance(boid, other) <= boid.seperationRange) {
      close_dx += boid.x - other.x;
      close_dy += boid.y - other.y;
    }
  }

  float dvx = close_dx * boid.avoidFactor;
  float dvy = close_dy * boid.avoidFactor;
  return {dvx, dvy};
}

// Reacts to other boids with visualRange. Returns the new velocity vector after seperation.
std::pair<float, float> align(const Roid& boid, const std::vector<Roid>& flock) {
  float vx_avg = 0;
  float vy_avg = 0;
  int neighbors = 0;
  for (const Roid& other : flock) {
    if (&boid == &other) continue;
    if (getDistance(boid, other) <= boid.visualRange) {
      vx_avg += other.vx;
      vy_avg += other.vy;
      neighbors++;
    }
  }
  if (neighbors > 0) {
    vx_avg = vx_avg / neighbors;
    vy_avg = vy_avg / neighbors;
  }

  float dvx = (vx_avg - boid.vx) * boid.alignmentFactor;
  float dvy = (vy_avg - boid.vy) * boid.alignmentFactor;
  return {dvx, dvy};
}

// Reacts to other boids with visualRange. Returns the new velocity vector after seperation.
std::pair<float, float> gather(const Roid& boid, const std::vector<Roid>& flock) {
  float x_avg = 0;
  float y_avg = 0;
  int neighbors = 0;
  for (const Roid& other : flock) {
    if (&boid == &other) continue;
    if (getDistance(boid, other) <= boid.visualRange) {
      x_avg += other.x;
      y_avg += other.y;
      neighbors++;
    }
  }

  if (neighbors > 0) {
    x_avg = x_avg / neighbors;
    y_avg = y_avg / neighbors;
  }

  float dx = (x_avg - boid.x) * boid.gatheringFactor;
  float dy = (y_avg - boid.y) * boid.gatheringFactor;
  return {dx, dy};
}

// Reacts to the edges of the boids margins. Returns the NEW VELOCITY (not DELTA).
std::pair<float, float> avoidMargin(const Roid& boid, int xBound, int yBound) {
  float nvx = boid.vx;
  float nvy = boid.vy;

  if (boid.x < 0 + boid.margins[0]) nvx += boid.turnFactor;
  if (boid.x > xBound - boid.margins[1]) nvx -= boid.turnFactor;
  if (boid.y < 0 + boid.margins[2]) nvy += boid.turnFactor;
  if (boid.y > yBound - boid.margins[3]) nvy -= boid.turnFactor;

  return {nvx, nvy};
}

std::pair<float, float> checkSpeed(const Roid& boid) {
  float speed = sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
  float vx_new = boid.vx;
  float vy_new = boid.vy;
  if (speed == 0) return {boid.vx + 2, boid.vy + 2};
  if (speed > boid.maxSpeed) {
    vx_new = (boid.vx / speed) * boid.maxSpeed;
    vy_new = (boid.vy / speed) * boid.maxSpeed;
  } else if (speed < boid.minSpeed) {
    vx_new = (boid.vx / speed) * boid.minSpeed;
    vy_new = (boid.vy / speed) * boid.minSpeed;
  }
  return {vx_new, vy_new};
}

void processRoids(World& world) {
  // Tick each boid's clock
  for (Roid& boid : world.boids) {
    // Seperation
    std::pair<float, float> deltaV = seperate(boid, world.boids);
    boid.vx += deltaV.first;
    boid.vy += deltaV.second;
    // Alignment
    deltaV = align(boid, world.boids);
    boid.vx += deltaV.first;
    boid.vy += deltaV.second;
    // Gathering
    deltaV = gather(boid, world.boids);
    boid.vx += deltaV.first;
    boid.vy += deltaV.second;
    // Avoid edge
    std::pair<float, float> newV = avoidMargin(boid, world.xBound, world.yBound);
    boid.vx = newV.first;
    boid.vy = newV.second;
    // Check speed
    newV = checkSpeed(boid);
    boid.vx = newV.first;
    boid.vy = newV.second;
    // Update position
    boid.x = boid.x + boid.vx;
    boid.y = boid.y + boid.vy;

    //std::cout << "New Pos: (";
    //std::cout << boid.x << ", ";
    //std::cout << boid.y << ")" << std::endl;
    //std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

// Should world be limited to this scope??
void mainLoop(World& world) {
  InitWindow(world.xBound, world.yBound, "Boids");
  SetTargetFPS(60);
  while (!WindowShouldClose()) {
    processRoids(world);
    drawRoids(world);
  }

  CloseWindow();
}

void run(const Options& options) {
 World world;
 world.xBound = 900;
 world.yBound = 900;

 for (int i = 0; i < options.numRoids; i++) {
   // Random position across screen
   float x = getRandom(50, 750);
   float y = getRandom(50, 750);

   // Random velocity with proper direction and speed
   float angle = getRandom(0, 6.28318530718); // 0 to 2π radians
   float speed = getRandom(options.minSpeed, options.maxSpeed);
   float vx = speed * std::cos(angle);
   float vy = speed * std::sin(angle);

   Roid boid{
     x, y,     // Position
     vx, vy,   // Velocity components
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

 mainLoop(world);
}

int main() {
  Options options;
  options.numRoids = 500;
  options.maxSpeed = 3;
  options.minSpeed = 2;
  options.maxTurnFactor = 0.2;
  options.minTurnFactor = 0.2;
  options.minVisualRange = 20;
  options.maxVisualRange = 20;

  run(options);
  return 0;
}
