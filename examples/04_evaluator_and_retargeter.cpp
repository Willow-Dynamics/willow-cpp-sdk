/**
 * Compile with: g++ -std=c++14 04_evaluator_and_retargeter.cpp -o ex04
 */
#include "../include/willow.hpp"
#include <iostream>

using namespace willow;

int main() {
    std::cout << "Starting Example 4: 3D Physics Evaluator & Retargeter\n\n";

    // Simulated Delta Time (~30 FPS). 
    // IMPORTANT: In production, dynamically calculate 'dt' per-frame using 
    // std::chrono::high_resolution_clock to prevent kinematic stutter/noise.
    float dt = 0.033f; 

    // ========================================================================
    // 1. EVALUATOR: 3D KINEMATICS
    // Demonstrating the calculation of Eulerian smoothness and explosive power
    // ========================================================================
    std::cout << "--- Kinematics (Circular Arc Test) ---\n";
    
    // Simulating an arm sweeping in a 90-degree arc over 3 frames
    Point3D p0 = {0.0f, 0.0f, 0.0f, 1.0f};
    Point3D p1 = {1.0f, 0.0f, 0.0f, 1.0f};
    Point3D p2 = {1.0f, 1.0f, 0.0f, 1.0f};
    Point3D p3 = {0.0f, 1.0f, 0.0f, 1.0f};

    // First Derivative (Velocity Vectors)
    Point3D v1 = Evaluator::calculate_velocity(p0, p1, dt);
    Point3D v2 = Evaluator::calculate_velocity(p1, p2, dt);
    Point3D v3 = Evaluator::calculate_velocity(p2, p3, dt);
    
    // Second Derivative (Acceleration Vectors - Captures centripetal force)
    Point3D a1 = Evaluator::calculate_acceleration(v1, v2, dt);
    Point3D a2 = Evaluator::calculate_acceleration(v2, v3, dt);

    // Third Derivative (Jerk Vector - Evaluates smoothness/inefficiency)
    Point3D jerk_vec = Evaluator::calculate_jerk(a1, a2, dt);

    // Extract scalar magnitudes for telemetry and UI dashboards
    float absolute_accel = Evaluator::get_magnitude(a2);
    float absolute_jerk  = Evaluator::get_magnitude(jerk_vec);

    std::cout << "Absolute Acceleration: " << absolute_accel << " m/s^2\n";
    std::cout << "Absolute Jerk:         " << absolute_jerk  << " m/s^3\n\n";


    // ========================================================================
    // 2. RETARGETER: GIMBAL LOCK SAFEGUARD TEST
    // Translating absolute 3D points into rotational joint quaternions
    // ========================================================================
    std::cout << "--- Retargeter (Colinear/Gimbal Lock Safeguard) ---\n";

    Point3D shoulder = {0.0f, 0.0f, 0.0f, 1.0f};
    Point3D elbow    = {0.0f, 1.0f, 0.0f, 1.0f}; // Arm pointing straight UP
    Point3D up_ref   = {0.0f, 1.0f, 0.0f, 1.0f}; // Standard World UP vector

    // By pointing straight up (matching the up_ref), the Cross Product is {0,0,0}.
    // A naive physics engine will crash with NaNs. The Willow V5.1 SDK dynamically
    // swaps the up_ref to the X-axis internally to prevent basis matrix collapse.
    
    Quaternion q = Retargeter::compute_lookat_rotation(shoulder, elbow, up_ref);
    
    std::cout << "Safeguarded Quaternion (w, x, y, z): " 
              << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "\n";

    return 0;
}