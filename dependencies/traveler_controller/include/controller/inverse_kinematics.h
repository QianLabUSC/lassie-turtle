#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#define _USE_MATH_DEFINES

#include <cmath>
#include <utility>
#include <cstdio>
#include <iostream>

#include "../proxy/control_data.h"   // for XY_pair, L1/L2/L3, MIN_EXT/MAX_EXT, etc.

// ------------------------------------------------------------
// Kinematics
// ------------------------------------------------------------

// gamma from leg length
void getGamma(float L, float& gamma);

// X,Y  ->  L,theta,gamma
void physicalToAbstract(float X, float Y, float& L, float& theta, float& gamma);

// X,Y  ->  theta,gamma  (optionally clamp XY into workspace first)
void physicalToAbstract(float X, float Y, float& theta, float& gamma, bool clamp = false);

// L,Theta  ->  x,y
void abstractToPhysical(float L, float Theta, float& x, float& y);
void abstractToPhysical(float L, float Theta, XY_pair& point);

// ------------------------------------------------------------
// Linear segment interpolation
// ------------------------------------------------------------

// time-based start/stop form
void linearTraj(float t, float t_start, float vel, XY_pair A, XY_pair B, float& X, float& Y);

// relative-time form (returns true when segment is complete)
bool linearTraj(float t_rel, float vel, XY_pair A, XY_pair B, float& X, float& Y);

// relative-time form with Toe feedback + threshold
bool linearTraj(float t_rel, float vel, XY_pair A, XY_pair B,
                XY_pair ToeXY, float& X, float& Y, float threshold = 0.01f);

// ------------------------------------------------------------
// Workspace helpers
// ------------------------------------------------------------

// clamp XY into [MIN_EXT, MAX_EXT] radial band; returns true if already valid
bool clamp_XY(float& x, float& y, float L = 0.0f);
bool clamp_XY(XY_pair& P, float L = 0.0f);

// simple utility used by the parser for bookkeeping
float distance(XY_pair A, XY_pair B);

#endif // INVERSE_KINEMATICS_H
