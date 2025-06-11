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
  std::vector<Roid> roids;
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
  for (const Roid& roid : world.roids) {
    DrawCircle(roid.x, roid.y, 5, BLACK);
  }

  EndDrawing();
}

double getDistance(const Roid& roid, const Roid& other) {
  double dx = roid.x - other.x;
  double dy = roid.y - other.y;
  return std::sqrt(dx * dx + dy * dy);
}

// Reacts to other roids with seperationRange. Returns the new velocity vector after seperation.
std::pair<double, double> seperate(const Roid& roid, const std::vector<Roid>& flock) {
  double close_dx = 0;
  double close_dy = 0;
  for (const Roid& other : flock) {
    if (&roid == &other) continue;
    if (getDistance(roid, other) <= roid.seperationRange) {
      close_dx += roid.x - other.x;
      close_dy += roid.y - other.y;
    }
  }

  double dvx = close_dx * roid.avoidFactor;
  double dvy = close_dy * roid.avoidFactor;
  return {dvx, dvy};
}

// Reacts to other roids with visualRange. Returns the new velocity vector after seperation.
std::pair<double, double> align(const Roid& roid, const std::vector<Roid>& flock) {
  double vx_avg = 0;
  double vy_avg = 0;
  int neighbors = 0;
  for (const Roid& other : flock) {
    if (&roid == &other) continue;
    if (getDistance(roid, other) <= roid.visualRange) {
      vx_avg += other.vx;
      vy_avg += other.vy;
      neighbors++;
    }
  }
  if (neighbors > 0) {
    vx_avg = vx_avg / neighbors;
    vy_avg = vy_avg / neighbors;
  }

  double dvx = (vx_avg - roid.vx) * roid.alignmentFactor;
  double dvy = (vy_avg - roid.vy) * roid.alignmentFactor;
  return {dvx, dvy};
}

// Reacts to other roids with visualRange. Returns the new velocity vector after seperation.
std::pair<double, double> gather(const Roid& roid, const std::vector<Roid>& flock) {
  double x_avg = 0;
  double y_avg = 0;
  int neighbors = 0;
  for (const Roid& other : flock) {
    if (&roid == &other) continue;
    if (getDistance(roid, other) <= roid.visualRange) {
      x_avg += other.x;
      y_avg += other.y;
      neighbors++;
    }
  }

  if (neighbors > 0) {
    x_avg = x_avg / neighbors;
    y_avg = y_avg / neighbors;
  }

  double dx = (x_avg - roid.x) * roid.gatheringFactor;
  double dy = (y_avg - roid.y) * roid.gatheringFactor;
  return {dx, dy};
}

// Reacts to the edges of the roids margins. Returns the NEW VELOCITY (not DELTA).
std::pair<double, double> avoidMargin(const Roid& roid, int xBound, int yBound) {
  double nvx = roid.vx;
  double nvy = roid.vy;

  if (roid.x < 0 + roid.xMargin.first) nvx += roid.turnFactor;
  if (roid.x > xBound - roid.xMargin.second) nvx -= roid.turnFactor;
  if (roid.y < 0 + roid.yMargin.first) nvy += roid.turnFactor;
  if (roid.y > yBound - roid.yMargin.second) nvy -= roid.turnFactor;

  return {nvx, nvy};
}

std::pair<double, double> checkSpeed(const Roid& roid) {
  double speed = sqrt(roid.vx * roid.vx + roid.vy * roid.vy);
  double vx_new = roid.vx;
  double vy_new = roid.vy;
  if (speed == 0) return {roid.vx + 2, roid.vy + 2};
  if (speed > roid.maxSpeed) {
    vx_new = (roid.vx / speed) * roid.maxSpeed;
    vy_new = (roid.vy / speed) * roid.maxSpeed;
  } else if (speed < roid.minSpeed) {
    vx_new = (roid.vx / speed) * roid.minSpeed;
    vy_new = (roid.vy / speed) * roid.minSpeed;
  }
  return {vx_new, vy_new};
}

void processRoids(World& world) {
  // Tick each roid's clock
  for (Roid& roid : world.roids) {
    // Seperation
    std::pair<double, double> deltaV = seperate(roid, world.roids);
    roid.vx += deltaV.first;
    roid.vy += deltaV.second;
    // Alignment
    deltaV = align(roid, world.roids);
    roid.vx += deltaV.first;
    roid.vy += deltaV.second;
    // Gathering
    deltaV = gather(roid, world.roids);
    roid.vx += deltaV.first;
    roid.vy += deltaV.second;
    // Avoid edge
    std::pair<double, double> newV = avoidMargin(roid, world.xBound, world.yBound);
    roid.vx = newV.first;
    roid.vy = newV.second;
    // Check speed
    newV = checkSpeed(roid);
    roid.vx = newV.first;
    roid.vy = newV.second;
    // Update position
    roid.x = roid.x + roid.vx;
    roid.y = roid.y + roid.vy;

    //std::cout << "New Pos: (";
    //std::cout << roid.x << ", ";
    //std::cout << roid.y << ")" << std::endl;
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

   Roid roid{
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

   world.roids.push_back(roid);
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
