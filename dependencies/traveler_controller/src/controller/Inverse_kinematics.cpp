#include "controller/inverse_kinematics.h"
#include <cmath>
#include <iostream>

using namespace std;

#define DEBUG
#define CONTROL_FREQ 100    // Hz

// Geometry constants (taken from old code)
const float L1 = 0.130f;   // flipper length
const float L2 = 0.079f;   // hip-to-ground height
const float L3 = 0.055f;   // offset below hip

/**
 * @brief Convert a Cartesian toe position into abstract leg coordinates.
 *
 * @param X toe x position
 * @param Y toe y position
 * @param theta output sweep angle
 * @param gamma output adduction angle
 * @param clamp clamping flag (currently unused)
 */
void physicalToAbstract(float X, float Y, float &theta, float &gamma, bool clamp)
{
    (void)clamp; // Suppress unused parameter warning
    
    float L = sqrtf(X*X + Y*Y);

    theta = atan2(X, Y);
    if (theta < -M_PI/2) {
        theta += 2*M_PI;  // wrap into range
    }

    // gamma from linkage geometry
    gamma = acosf((pow(L - L3, 2) + L1*L1 - L2*L2) / (2 * L1 * (L - L3)));
}

/**
 * @brief Convert abstract leg position (L, Theta) back into Cartesian (x,y).
 */
void abstractToPhysical(float L, float Theta, float &x, float &y)
{
    x = -L * sinf(Theta);
    y =  L * cosf(Theta);
}

void abstractToPhysical(float L, float Theta, XY_pair &point)
{
    point.x = -L * sinf(Theta);
    point.y =  L * cosf(Theta);
}

/**
 * @brief Linear interpolation between two waypoints.
 *
 * @param t_rel relative time since start of segment
 * @param vel   desired toe velocity
 * @param A     start point
 * @param B     end point
 * @param X,Y   output interpolated point
 * @return true if segment complete
 */

bool linearTraj(float t_rel, float vel, XY_pair A, XY_pair B, float &X, float &Y)
{
    float dx = B.x - A.x;
    float dy = B.y - A.y;
    float dist = sqrtf(dx*dx + dy*dy);
    
    if (dist < 1e-6f) {
        X = B.x;
        Y = B.y;
        return true;
    }
    
    float desired_dist = vel * t_rel;
    
    if (desired_dist >= dist) {
        X = B.x;
        Y = B.y;
        return true;
    }
    
    float t = desired_dist / dist;
    X = A.x + t * dx;
    Y = A.y + t * dy;
    
    return false;
}


void getGamma(float L, float& gamma) {
    // Implementation for getting gamma from length L
    gamma = acosf((L*L + L1*L1 - L2*L2) / (2 * L1 * L));
}

bool linearTraj(float t_rel, float vel, XY_pair A, XY_pair B, XY_pair ToeXY, float& X, float& Y, float threshold) {
    (void)ToeXY; 
    (void)threshold;
    return linearTraj(t_rel, vel, A, B, X, Y);
}

bool validPath(XY_pair A, XY_pair B) {
    // Check if path from A to B is valid
    float dist = distance(A, B);
    return dist > 0.0f && dist < 1.0f; // Example constraint
}

Point_Pair findSwingPoints(XY_pair A, XY_pair B) {
    // Find intermediate swing points between A and B
    Point_Pair result;
    result.A = A;
    result.B = B;
    return result;
}

bool inBounds(XY_pair ToeXY) {
    return inBounds(ToeXY.x, ToeXY.y);
}

bool inBounds(float x, float y) {
    // Check if point (x,y) is within workspace bounds
    float dist = sqrtf(x*x + y*y);
    return dist >= 0.11f && dist <= 0.22f; // Example workspace bounds
}

bool clamp_XY(XY_pair& P, float L) {
    (void)L; // Suppress unused parameter warning
    // Clamp point P to workspace bounds
    float dist = sqrtf(P.x*P.x + P.y*P.y);
    if (dist > 0.22f) {
        P.x = P.x * 0.22f / dist;
        P.y = P.y * 0.22f / dist;
        return true;
    }
    if (dist < 0.11f && dist > 0.0f) {
        P.x = P.x * 0.11f / dist;
        P.y = P.y * 0.11f / dist;
        return true;
    }
    return false;
}

float distance(XY_pair A, XY_pair B) {
    float dx = B.x - A.x;
    float dy = B.y - A.y;
    return sqrtf(dx*dx + dy*dy);
}

