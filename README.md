# Willow 5 Runtime: C++ Edge SDK

The **Willow C++ Edge SDK** is a dependency-free, header-only inference library designed for sub-millisecond execution of Zero-Shot Action Recognition directly on IoT, VR/AR Edge, and Robotic platforms.

It acts as the edge counterpart to the Willow Cloud Oracle and ensures absolute mathematical parity. 

## 📦 Modules Included
- **Client:** Secure RAM-only model provisioning via HTTPS.
- **Detector:** Continuous Subsequence DTW and algorithmic Non-Maximum Suppression (NMS).
- **Evaluator:** High-order physical derivative calculations (Velocity, Acceleration, Jerk).
- **Retargeter:** 3D Kinematic mapping and Quaternion calculations.
- **Transforms:** Bridge matrices for MediaPipe, Unity, and ROS.

## 🚀 Quickstart
Simply include `include/willow.hpp` in your C++14 (or higher) project. To enable the built-in HTTP Client, link `libcurl` and define `#define WILLOW_ENABLE_CURL`.

```cpp
#include "willow.hpp"

// 1. Provision Model (RAM-Only)
std::vector<uint8_t> buffer = fetch_from_cloud();
willow::Model model = willow::Model::load_from_memory(buffer);

// 2. Initialize Engine
willow::Detector detector(model);
willow::DetectionResult result;

// 3. Process Live Data
willow::Skeleton frame = get_75_point_skeleton();
if (detector.push_frame(frame, result)) {
    std::cout << "Action Detected! Confidence: " << result.confidence << "\n";
}
```

## 📚 Documentation
Please refer to the `docs/` folder for comprehensive architectural guidelines:
1.[Architecture & Best Practices](docs/architecture_and_best_practices.md)
2. [Topology & Transforms](docs/topology_and_transforms.md)
3. [Provisioning & DRM](docs/provisioning.md)
4. [Evaluator Physics](docs/evaluator_physics.md)
5.[Retargeter Kinematics](docs/retargeter_kinematics.md)

## 🛠️ Building Examples & Tests
```bash
# Run the Golden Vector Parity Test
g++ -std=c++14 tests/test_golden_vector.cpp -o test_willow
./test_willow

# Run Examples
g++ -std=c++14 examples/01_basic_detection.cpp -o ex01
./ex01
```