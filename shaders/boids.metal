#include <metal_stdlib>

struct Boid {
  float2 position; // x, y
  float2 velocity; // vx, vy

  float seperationRange;
  float avoidFactor;
  float visualRange;
  float alignmentFactor;
  float gatheringFactor;
  float turnFactor;
  float maxSpeed;
  float minSpeed;

  float4 margins; // {Left, Right, Bottom Top}
};

struct WorldParams {
  uint32_t numBoids;
  float xBound;
  float yBound;
};

kernel void updateBoids(device Boid* boids[[buffer(0)]],
    constant WorldParams& params [[buffer(1)]],
    uint id [[thread_position_in_grid]]) {
  if (id >= params.numBoids) return;

  Boid boid = boids[id];
  float2 newVelocity = boid.velocity;

  float2 seperation = float2(0.0);
  float2 alignmentSum = float2(0.0);
  float2 cohesionSum = float2(0.0);
  int neighbors = 0;

  for (uint i = 0; i < params.numBoids; i++) {
    if (i == id) continue; // Skip self

    Boid other = boids[i];
    float2 diff = boid.position - other.position; //SIMD?????
    float distance = metal::length(diff);

    if (distance <= boid.seperationRange && distance > 0) {
      seperation += diff;
    }

    if (distance <= boid.visualRange && distance >= 0) {
      alignmentSum += other.velocity;
      cohesionSum += other.position;
      neighbors++;
    }
  }

  newVelocity += seperation * boid.avoidFactor;

  if (neighbors > 0) {
    float2 avgVelocity = alignmentSum / float(neighbors);
    newVelocity += (avgVelocity - boid.velocity) * boid.alignmentFactor;

    float2 avgPosition = cohesionSum / float(neighbors);
    newVelocity += (avgPosition - boid.position) * boid.gatheringFactor;
  }

  if (boid.position.x < boid.margins.x) newVelocity.x += boid.turnFactor;
  if (boid.position.x > params.xBound - boid.margins.y) newVelocity.x -= boid.turnFactor;
  if (boid.position.y < boid.margins.z) newVelocity.y += boid.turnFactor;
  if (boid.position.y > params.yBound - boid.margins.w) newVelocity.y -= boid.turnFactor;

  float speed = metal::length(newVelocity);
  if (speed > boid.maxSpeed) {
    newVelocity = metal::normalize(newVelocity) * boid.maxSpeed;
  } else if (speed < boid.minSpeed && speed > 0) {
    newVelocity = metal::normalize(newVelocity) * boid.minSpeed;
  } else if (speed == 0) {
    newVelocity = float2(2.0, 2.0); // nudge
  }

  boids[id].velocity = newVelocity;
  boids[id].position = boid.position + newVelocity;

}
