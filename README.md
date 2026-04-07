# Willow 5 Runtime SDK (C++)

[![Version](https://img.shields.io/badge/version-5.4.0-blue.svg)](https://willowdynamics.com)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The **Willow 5 Runtime C++ SDK** is a dependency-free, header-only inference library designed for sub-millisecond execution of Zero-Shot Action Recognition directly on IoT, VR/AR Edge, and Robotic platforms.

While the Willow Cloud Oracle is used to *train and generate* proprietary Action Models, this C++ SDK allows you to execute those models locally in embedded environments. It acts as the high-performance bridge between Willow's biomechanical intelligence and your hardware.

## Core Capabilities
- **Dual Spatial Engines**: Natively decodes V40 (Scale-Invariant) and V41 (Absolute Physics/Metric) signatures.
- **Object Tracking (Node 76)**: Natively process human-object interactions (tools, balls) in a unified 3D metric space.
- **Zero-Allocation Memory**: Uses internal ring-buffers to guarantee deterministic execution times (< 1ms) without Heap fragmentation.
- **Sim-to-Real Retargeting**: Convert Computer Vision coordinates into ROS (Z-Up) and Unity formats natively.
- **Edge Physics**: Calculate Jerk, Acceleration, and Power without cloud latency using the ported Da Vinci engine.
- **DRM-Compliant**: Models can load securely into ephemeral RAM via `libcurl` to protect Intellectual Property.

---

## Spatial Engine Modes (New in Q2 2026)
The C++ SDK automatically parses the binary header of your `.int8` model to determine its mathematical routing:
- **Universal Models (Scale-Invariant)**: Evaluates structural form and sequence. Distances are mathematically normalized by torso length. Expects a `75-point` generic coordinate `Skeleton`.
- **Physics Models (Metric Space)**: Evaluates exact volumetric paths and physical velocities (m/s). Normalization is bypassed. Expects coordinates to be provided in absolute **Meters**, utilizing a `76-point` `Skeleton` where Index 75 represents the tracked reference object.

---

## 🚀 Quickstart

The SDK is completely header-only. Simply include `include/willow.hpp` in your C++14 (or higher) project. 
To enable the built-in HTTP Client for secure cloud provisioning, link `libcurl` and define `-DWILLOW_ENABLE_CURL`.

```cpp
#include "willow.hpp"

// 1. Provision Model (RAM-Only)
std::vector<uint8_t> buffer = fetch_from_cloud();
willow::Model model = willow::Model::load_from_memory(buffer);

// 2. Initialize Engine
willow::Detector detector(model);
willow::DetectionResult result;

// 3. Process Live Data
willow::Skeleton frame = get_76_point_skeleton();
if (detector.push_frame(frame, result)) {
    std::cout << "Action Detected! Confidence: " << result.confidence << "\n";
}
```

---

## 📚 Documentation
Please refer to the `docs/` folder for comprehensive architectural guidelines:
1. [Architecture & Best Practices](docs/architecture_and_best_practices.md) - Understand how the Willow system functions.
2. [Topology & Transforms](docs/topology_and_transforms.md) - The Willow 76-point joint index reference & ROS/Unity bridging.
3.[Provisioning & DRM](docs/provisioning.md) - How to fetch, cache, and manage secure `.int8` models via `libcurl`.
4. [Evaluator Physics](docs/evaluator_physics.md) - Scoring speed, smoothness, and impulse at the edge.
5. [Retargeter Kinematics](docs/retargeter_kinematics.md) - Extracting Quaternions for RL & Simulation.

## 🛠️ Building Examples & Tests
```bash
# Run the Golden Vector Parity Test (Verifies V40/V41 parsing)
g++ -std=c++14 tests/test_golden_vector.cpp -o test_willow
./test_willow

# Run Examples
g++ -std=c++14 examples/01_basic_detection.cpp -o ex01
./ex01
```

## Support & Licensing
The Willow Runtime is a premium commercial service. A **Partner License** is required to provision models.
- [Request a License](https://willowdynamics.com/pages/contact)
- [Technical Support](https://willowdynamics.com/pages/contact)

&copy; 2026 Willow Dynamics. All rights reserved.