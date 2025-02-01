#include <iostream>
#include <cmath>
#include "controller/inverse_kinematics.h"  // Adjust include path as needed

// A utility clamp function (in degrees) to ensure angles stay within servo/motor limits.
static float clamp_angle_deg(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

void boundingGAIT(turtle& turtle_, float t)
{
    // -----------------------------------------------------------------------
    // 1) Define durations for the four phases
    //    (Tune these numbers to match your desired timing.)
    // -----------------------------------------------------------------------
    float phase1_duration = 1.0f;  // Swing + partial insertion
    float phase2_duration = 0.8f;  // Finish insertion
    float phase3_duration = 1.2f;  // Stance
    float phase4_duration = 1.0f;  // Extraction

    // The total cycle time
    float total_period = phase1_duration
                       + phase2_duration
                       + phase3_duration
                       + phase4_duration;

    // Wrap time into [0, total_period] so it loops
    float t_mod = fmod(t, total_period);
    // If you track how many cycles have elapsed:
    turtle_.turtle_chassis.step_count = (t - t_mod) / total_period; 

    // -----------------------------------------------------------------------
    // 2) Define the angles in degrees for each important position
    //    (You can rename or reorder them to fit your flipper geometry.)
    // -----------------------------------------------------------------------
    // Sweeping angles (theta)
    float theta_start_deg   = -30.0f; // e.g. the “back” position 
    float theta_swing_deg   = +30.0f; // e.g. the “forward” position
    
    // Insertion angles (gamma)
    float gamma_extracted_deg  =   0.0f; // “up” or “out of mud”
    float gamma_partial_deg    = -15.0f; // partial insertion
    float gamma_full_deg       = -30.0f; // fully inserted

    // We'll define temporary variables that we'll fill in during each phase
    float left_theta  = 0.f;
    float left_gamma  = 0.f;
    float right_theta = 0.f;
    float right_gamma = 0.f;

    // -----------------------------------------------------------------------
    // 3) Phase 1: Swing + partial insertion (0 <= t_mod < phase1_duration)
    // -----------------------------------------------------------------------
    if (t_mod < phase1_duration)
    {
        float s = t_mod / phase1_duration;  // goes from 0..1
        turtle_.turtle_chassis.gait_state = 1;  // ID for debugging if needed

        // Theta: from theta_start_deg to theta_swing_deg
        left_theta  = theta_start_deg + (theta_swing_deg - theta_start_deg) * s;
        right_theta = -left_theta; // mirrored

        // Gamma: from gamma_extracted_deg to gamma_partial_deg
        left_gamma  = gamma_extracted_deg + (gamma_partial_deg - gamma_extracted_deg) * s;
        right_gamma = -left_gamma; // mirrored insertion
    }
    // -----------------------------------------------------------------------
    // 4) Phase 2: Finish insertion (phase1_duration <= t_mod < phase1+phase2)
    // -----------------------------------------------------------------------
    else if (t_mod < (phase1_duration + phase2_duration))
    {
        turtle_.turtle_chassis.gait_state = 2;
        float s_phase = (t_mod - phase1_duration) / phase2_duration; // 0..1 in Phase 2

        // Theta: Let's assume we hold it at the final swing position from Phase 1
        left_theta  = theta_swing_deg;
        right_theta = -theta_swing_deg;

        // Gamma: from partial insertion to full insertion
        left_gamma  = gamma_partial_deg + (gamma_full_deg - gamma_partial_deg) * s_phase;
        right_gamma = -left_gamma;
    }
    // -----------------------------------------------------------------------
    // 5) Phase 3: Stance (phase1+phase2 <= t_mod < phase1+phase2+phase3)
    // -----------------------------------------------------------------------
    else if (t_mod < (phase1_duration + phase2_duration + phase3_duration))
    {
        turtle_.turtle_chassis.gait_state = 3;
        float s_phase = (t_mod - (phase1_duration + phase2_duration)) / phase3_duration;

        // Typically, in stance, the flipper is fully inserted (gamma_full_deg).
        // Possibly the sweeping angle might be a small scull or might remain fixed 
        // at the forward position. For simplicity, let's hold them constant here:

        left_theta  = theta_swing_deg;  // hold
        right_theta = -theta_swing_deg; // hold
        left_gamma  = gamma_full_deg;   // fully inserted 
        right_gamma = -gamma_full_deg;
        
        // (If you want to do a small “sculling” motion in stance, 
        //  you could do an interpolation from +X deg to +Y deg here.)
    }
    // -----------------------------------------------------------------------
    // 6) Phase 4: Extraction (phase1+phase2+phase3 <= t_mod < total_period)
    // -----------------------------------------------------------------------
    else
    {
        turtle_.turtle_chassis.gait_state = 4;
        float t_phase_start = phase1_duration + phase2_duration + phase3_duration;
        float s_phase = (t_mod - t_phase_start) / phase4_duration; // 0..1 in Phase 4

        // Let's say we want to come back from the swing_deg to the start_deg
        float theta_start = theta_swing_deg;
        float theta_end   = theta_start_deg;
        left_theta  = theta_start + (theta_end - theta_start) * s_phase;
        right_theta = -left_theta;

        // Gamma: from full insertion back to extracted (0 deg)
        float gamma_start = gamma_full_deg;
        float gamma_end   = gamma_extracted_deg;
        left_gamma  = gamma_start + (gamma_end - gamma_start) * s_phase;
        right_gamma = -left_gamma;
    }

    // -----------------------------------------------------------------------
    // 7) Clamp the angles if necessary
    // -----------------------------------------------------------------------
    left_theta   = clamp_angle_deg(left_theta,  -60.f,  60.f);
    right_theta  = clamp_angle_deg(right_theta, -60.f,  60.f);
    left_gamma   = clamp_angle_deg(left_gamma,  -45.f,  45.f);
    right_gamma  = clamp_angle_deg(right_gamma, -45.f,  45.f);

    // -----------------------------------------------------------------------
    // 8) Send the final commands to motors/servos
    //    (Degrees for a typical servo, or convert to radians if needed.)
    // -----------------------------------------------------------------------
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position  = left_theta;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = right_theta;
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position  = left_gamma;
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = right_gamma;

    // // If you also have radian-based commands:
    // turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position  
    //     = -left_theta  / 360.f;
    // turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position 
    //     = -right_theta / 360.f;
    // turtle_.turtle_control.left_adduction.set_input_position_radian.input_position  
    //     = -left_gamma  / 360.f;
    // turtle_.turtle_control.right_adduction.set_input_position_radian.input_position 
    //     = -right_gamma / 360.f;

    // -----------------------------------------------------------------------
    // 9) (Optional) Debug print
    // -----------------------------------------------------------------------
    // std::cout << "[BOUNDING GAIT] Phase=" << turtle_.turtle_chassis.gait_state
    //           << "  L(theta,gamma)=(" << left_theta  << "," << left_gamma  << ")"
    //           << "  R(theta,gamma)=(" << right_theta << "," << right_gamma << ")\n";
}