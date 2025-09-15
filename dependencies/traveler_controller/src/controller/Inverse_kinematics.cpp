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
 * @param clamp ignored for now (we don’t clamp)
 */
void physicalToAbstract(float X, float Y, float &theta, float &gamma, bool clamp)
{
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

    if (dist == 0.0f) {
        X = A.x;
        Y = A.y;
        return true;
    }

    float scalar = vel / dist;
    X = scalar * dx * t_rel + A.x;
    Y = scalar * dy * t_rel + A.y;

    float actual_dist = sqrtf((X - A.x)*(X - A.x) + (Y - A.y)*(Y - A.y));
    return (actual_dist >= dist);
}
