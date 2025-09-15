#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <cmath>
#include <iostream>
#include "../proxy/control_data.h"  // XY_pair, Point_Pair, Waypoint, Traveler, etc.

// ------------------------------------------------------------
// Core kinematics
// ------------------------------------------------------------
void getGamma(float L, float& gamma);

void physicalToAbstract(float X, float Y, float& theta, float& gamma, bool clamp = false);
void abstractToPhysical(float L, float Theta, XY_pair& point);

// ------------------------------------------------------------
// Linear segment interpolation
// ------------------------------------------------------------
bool linearTraj(float t_rel, float vel, XY_pair A, XY_pair B, float& X, float& Y);
bool linearTraj(float t_rel, float vel, XY_pair A, XY_pair B,
                XY_pair ToeXY, float& X, float& Y, float threshold = 0.01f);

// ------------------------------------------------------------
// Path helpers
// ------------------------------------------------------------
bool validPath(XY_pair A, XY_pair B);
Point_Pair findSwingPoints(XY_pair A, XY_pair B);

// ------------------------------------------------------------
// Workspace helpers
// ------------------------------------------------------------
bool inBounds(XY_pair ToeXY);
bool inBounds(float x, float y);

bool clamp_XY(XY_pair& P, float L = 0.0f);
float distance(XY_pair A, XY_pair B);

// ------------------------------------------------------------
// Params
// ------------------------------------------------------------
struct WorkspaceTraversalParams {
    float max_ext = 0.22f;
    float min_ext = 0.11f;
    double min_theta = 0.6f;
    double max_theta = 4.1f;
    double d_theta = 0.008f;
    float d_L = 0.0002f;
    float L_step = 0.01f;

    float curr_ext = max_ext;
    float curr_theta = min_theta;
    bool cw = true;
    bool run = false;
    int counter = 0;
    bool shorten_leg = false;
    float next_ext = 0.0f;
    const int cycle_len = 1;
    const int measure_time = 0;
};

#endif // INVERSE_KINEMATICS_H
