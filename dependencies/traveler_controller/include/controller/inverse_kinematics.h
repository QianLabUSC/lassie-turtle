#pragma once

#include <tuple>
#include <utility>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include "proxy/control_data.h"
// Global Debug Flag
const float l_1 = 0.1;
const float l_2 = 0.2;

// void GetAngles(LegConfig leg, float& angle_0, float& angle_1);

void GetGamma(float L, float theta, float& gamma);

void PhysicalToAbstract(float X1, float Y1, float X2, float Y2, float &theta1, float &gamma1, float &beta1, float &theta2, float &gamma2, float &beta2);
void checkright(float &gamma2, float &beta2, float &theta2);
void checkleft(float &gamma1, float &beta1, float &theta1);
// void PhysicalToAbstract(float X, float Y, float &theta, float &gamma);

// void PhysicalToAbstract(LegConfig leg, float& L, float& theta, float& gamma);

void AbstractToPhysical(float L, float Theta, float gamma, float& x, float& y);

// void MoveToPosition(LegConfig leg, float t);

void RadialTrajectory(float t, struct RadialGaitParams gait, float& X, float& Y);

// void RadialLegMovement(LegConfig leg, float t, struct RadialGaitParams gait, float& theta, float& gamma);
void RightTriangleTrajectory(float t, struct RightTriangularGaitParams gait, float& X, float& Y);
void TriangularTrajectory(float t, struct TriangularGaitParams gait, float& X, float& Y);
void boundingGAIT(turtle &turtle_, float t);
// Triangular Trajectory Helpers :
void HorizontalStep(float t, struct TriangularGaitParams gait, float& X, float& Y);
void SwingAngle(float t, struct TriangularGaitParams gait, float& X, float& Y);
void RadialMove(float t, struct TriangularGaitParams gait, float& X, float& Y);

bool inBounds(float Gamma, float Theta, float L);

bool inBounds(float x, float y);

struct OvalParams {
    float period_down = 3.0f; // Initial Length of leg
    float period_up = 3.0f; // Final leg length
    float vertical_range = 0.01f; // Angle of radial movement (half of the whole range)
    float horizontal_range = 0.05f; // Frequency of one movement cycle (Hz) (half of the whole range)
};

struct Rectangle_Params {
    float period_down = 3.0f; // time
    float period_up = 3.0f; // time
    float period_left = 3.0f; // time
    float period_right = 3.0f; // time
    float vertical_range = 0.01f; // insertion depth (length)
    float horizontal_range = 0.03f; // half of the whole range (length)
    float period_waiting_time = 0.5f; // time
    float wiggle_length = 0.2f;
    float wiggle_frequency = 10.0;
};

