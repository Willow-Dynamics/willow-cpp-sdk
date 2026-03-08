# Evaluator: Physical Derivatives

The `Evaluator` module computes the higher-order physical derivatives required by the Willow Cloud Oracle to determine action intensity, efficiency, and fluidity ("Eulerian smoothness").

While the `Detector` identifies the action, the `Evaluator` grades the **quality** of the movement.

## 1. Mathematical Hierarchy
The Evaluator operates on a standard kinematic hierarchy, but crucially, it performs all calculations using **3D Vectors**, not scalar speeds. This ensures that directional changes (centripetal acceleration) are correctly captured.

* **Velocity (v = d/dt):** The rate of change of position. Returned as a `Point3D` vector.
* **Acceleration (a = d²/dt²):** The rate of change of velocity. High acceleration indicates explosive power. Returned as a `Point3D` vector.
* **Jerk (j = d³/dt³):** The rate of change of acceleration. High jerk indicates a lack of smoothness or efficiency (wasted energy). Returned as a `Point3D` vector.

## 2. Implementation
The `Evaluator` class provides static helper functions. Because the C++ SDK is designed to be lightweight and stateless, **you must maintain your own historical window** (e.g., a ring buffer or a simple variable cache) of previous frames to compute these vectors over time.

### Example Calculation
Assuming a stable delta time (`dt`) of 0.033s (30 FPS):

```cpp
float dt = 0.033f; 

// 1. Get Positions from History
willow::Point3D p0 = history_buffer[frame - 2];
willow::Point3D p1 = history_buffer[frame - 1];
willow::Point3D p2 = current_frame_point;

// 2. Compute Velocities (First Derivative - Vectors)
willow::Point3D v1 = willow::Evaluator::calculate_velocity(p0, p1, dt);
willow::Point3D v2 = willow::Evaluator::calculate_velocity(p1, p2, dt);

// 3. Compute Acceleration (Second Derivative - Vector)
// Calculates true 3D acceleration (linear + centripetal)
willow::Point3D accel_vec = willow::Evaluator::calculate_acceleration(v1, v2, dt);

// 4. Compute Jerk (Third Derivative - Vector)
// Requires a previous acceleration vector from the history buffer
willow::Point3D prev_accel_vec = history_accel[frame - 1];
willow::Point3D jerk_vec = willow::Evaluator::calculate_jerk(prev_accel_vec, accel_vec, dt);

// 5. Extract Scalar Metrics for Analytics
float current_jerk_mag = willow::Evaluator::get_magnitude(jerk_vec);

std::cout << "Current Jerk: " << current_jerk_mag << " m/s^3\n";
```