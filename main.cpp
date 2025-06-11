#include <vector>
#include <random>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <raylib.h>

struct Roid {
  double x;
  double y;
  double vx; // velocity
  double vy; // velocity

  double seperationRange;
  double avoidFactor;
  double visualRange;
  double alignmentFactor;
  double gatheringFactor;
  double turnFactor;

  double maxSpeed;
  double minSpeed;

  std::pair<int, int> xMargin;
  std::pair<int, int> yMargin;
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

  double minSepRange;
  double maxSepRange;

  double minAvoidFactor;
  double maxAvoidFactor;

  double minVisualRange;
  double maxVisualRange;

  double minAlignmentFactor;
  double maxAlignmentFactor;

  double minGatheringFactor;
  double maxGatheringFactor;

  double minTurnFactor;
  double maxTurnFactor;

  double minMaxSpeed;
  double maxMaxSpeed;

  std::pair<int, int> minXMargin; // {minLeftMargin, minRightMargin}
  std::pair<int, int> maxXMargin; // {maxLeftMargin, maxRightMargin}
  std::pair<int, int> minYMargin; // {minBottomMargin, minTopMargin}
  std::pair<int, int> maxYMargin; // {maxBottomMargin, maxTopMargin}
};

double getRandom(double min, double max) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(min, max);
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

double getDistance(const Roid& boid, const Roid& other) {
  double dx = boid.x - other.x;
  double dy = boid.y - other.y;
  return std::sqrt(dx * dx + dy * dy);
}

// Reacts to other boids with seperationRange. Returns the new velocity vector after seperation.
std::pair<double, double> seperate(const Roid& boid, const std::vector<Roid>& flock) {
  double close_dx = 0;
  double close_dy = 0;
  for (const Roid& other : flock) {
    if (&boid == &other) continue;
    if (getDistance(boid, other) <= boid.seperationRange) {
      close_dx += boid.x - other.x;
      close_dy += boid.y - other.y;
    }
  }

  double dvx = close_dx * boid.avoidFactor;
  double dvy = close_dy * boid.avoidFactor;
  return {dvx, dvy};
}

// Reacts to other boids with visualRange. Returns the new velocity vector after seperation.
std::pair<double, double> align(const Roid& boid, const std::vector<Roid>& flock) {
  double vx_avg = 0;
  double vy_avg = 0;
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

  double dvx = (vx_avg - boid.vx) * boid.alignmentFactor;
  double dvy = (vy_avg - boid.vy) * boid.alignmentFactor;
  return {dvx, dvy};
}

// Reacts to other boids with visualRange. Returns the new velocity vector after seperation.
std::pair<double, double> gather(const Roid& boid, const std::vector<Roid>& flock) {
  double x_avg = 0;
  double y_avg = 0;
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

  double dx = (x_avg - boid.x) * boid.gatheringFactor;
  double dy = (y_avg - boid.y) * boid.gatheringFactor;
  return {dx, dy};
}

// Reacts to the edges of the boids margins. Returns the NEW VELOCITY (not DELTA).
std::pair<double, double> avoidMargin(const Roid& boid, int xBound, int yBound) {
  double nvx = boid.vx;
  double nvy = boid.vy;

  if (boid.x < 0 + boid.xMargin.first) nvx += boid.turnFactor;
  if (boid.x > xBound - boid.xMargin.second) nvx -= boid.turnFactor;
  if (boid.y < 0 + boid.yMargin.first) nvy += boid.turnFactor;
  if (boid.y > yBound - boid.yMargin.second) nvy -= boid.turnFactor;

  return {nvx, nvy};
}

std::pair<double, double> checkSpeed(const Roid& boid) {
  double speed = sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
  double vx_new = boid.vx;
  double vy_new = boid.vy;
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
    std::pair<double, double> deltaV = seperate(boid, world.boids);
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
    std::pair<double, double> newV = avoidMargin(boid, world.xBound, world.yBound);
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
  InitWindow(800, 800, "Boids");
  SetTargetFPS(60);
  while (!WindowShouldClose()) {
    processRoids(world);
    drawRoids(world);
  }

  CloseWindow();
}

void run(const Options& options) {
 World world;
 world.xBound = 800;
 world.yBound = 800;

 for (int i = 0; i < options.numRoids; i++) {
   // Random position across screen
   double x = getRandom(50, 750);
   double y = getRandom(50, 750);

   // Random velocity with proper direction and speed
   double angle = getRandom(0, 6.28318530718); // 0 to 2π radians
   double speed = getRandom(options.minSpeed, options.maxSpeed);
   double vx = speed * std::cos(angle);
   double vy = speed * std::sin(angle);

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
     {getRandom(options.minXMargin.first, options.maxXMargin.first),
      getRandom(options.minXMargin.second, options.maxXMargin.second)}, // xMargin
     {getRandom(options.minYMargin.first, options.maxYMargin.first),
      getRandom(options.minYMargin.second, options.maxYMargin.second)}  // yMargin
   };

   world.boids.push_back(boid);
 }

 mainLoop(world);
}

int main() {
  Options options;
  options.numRoids = 900;
  options.maxSpeed = 3;
  options.minSpeed = 2;
  options.maxTurnFactor = 0.2;
  options.minTurnFactor = 0.2;
  options.minVisualRange = 20;
  options.maxVisualRange = 20;
  options.minXMargin = {100, 100};
  options.maxXMargin = {100, 100};

  options.minYMargin = {100, 100};
  options.maxYMargin = {100, 100};

  run(options);
  return 0;
}
