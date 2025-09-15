#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <cmath>
#include <utility>
#include <cstdio>
#include <iostream>

#include "proxy/control_data.h" // XY_pair, Waypoint, turtle, etc.

// ------------------------------------------------------------
// Kinematics (minimal forms)
// ------------------------------------------------------------

// X,Y  ->  theta,gamma  (gamma fixed at 0 here)
void physicalToAbstract(float X, float Y, float& theta, float& gamma, bool clamp = false);

// L,Theta  ->  x,y (helpers kept for completeness)
void abstractToPhysical(float L, float Theta, float& x, float& y);
void abstractToPhysical(float L, float Theta, XY_pair& point);

// ------------------------------------------------------------
// Linear segment interpolation
// ------------------------------------------------------------

// time-based start/stop form
void linearTraj(float t, float t_start, float vel, XY_pair A, XY_pair B, float& X, float& Y);

// relative-time form (returns true when segment is complete)
bool linearTraj(float t_rel, float vel, XY_pair A, XY_pair B, float& X, float& Y);

// relative-time form with Toe feedback + threshold (ignored here)
bool linearTraj(float t_rel, float vel, XY_pair A, XY_pair B,
                XY_pair ToeXY, float& X, float& Y, float threshold = 0.01f);

// ------------------------------------------------------------
// Workspace helpers (no-op implementations)
// ------------------------------------------------------------

bool clamp_XY(float& x, float& y, float L = 0.0f);
bool clamp_XY(XY_pair& P, float L = 0.0f);

// utility
float distance(XY_pair A, XY_pair B);

#endif // INVERSE_KINEMATICS_H



