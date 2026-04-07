# Willow 5 Runtime: Architecture & Engineering Best Practices

Welcome to the Willow 5 Runtime. If you are an integration engineer, roboticist, or data scientist, this document is your definitive guide to understanding how the Willow system functions, where its boundaries lie, and how to architect resilient applications around it.

At a high level, the Willow SDK is a **Local Execution Environment**. We provide the mathematical kernels and pre-compiled AI models; you provide the raw 3D spatial data (from cameras, LiDAR, or XR headsets).

To integrate this SDK successfully, you must understand the distinction between the **Pristine Model** (created in our Cloud Oracle) and the **Messy Reality** (the live data streaming from your edge hardware).

---

## Part 1: Core System Architecture

### 1. The Data Contract: Sensor Agnosticism
The Willow SDK does not contain a computer vision engine. It does not know what a camera or a pixel is. It expects an input of a 75 or 76-point `Skeleton` array (`std::vector<Point3D>`)—a 3D point cloud mapped to the Willow Topology.

Because we decouple the vision engine from the physics engine, you can feed Willow data from a $1,000 depth sensor on a robot, a Meta Quest 3 headset, or a mobile phone running a TFLite model.

### 2. Dual Spatial Engines: Universal vs. Physics
When you pass a skeleton into the SDK, the `extract_rdm()` kernel converts the raw X, Y, Z coordinates into a **Relative Distance Matrix (RDM)**. The math routing depends on your model:

*   **Universal Models (Scale-Invariant):** The engine dynamically calculates the 3D distance between the shoulders and hips to establish Torso Length. Every other distance in the body is divided by this Torso Length. A hand moving to the shoulder looks mathematically identical regardless of the subject's absolute height.
*   **Physics Models (Metric Space):** The normalization scaling is completely bypassed. The engine calculates absolute physical distances in meters. This mode natively supports Node 76 (Tracked Objects), allowing you to verify physical interactions between the human and their environment.

### 3. Continuous DTW & Non-Maximum Suppression (NMS)
To achieve zero-latency action recognition, the SDK runs **Continuous Dynamic Time Warping (DTW)**. 
*   Instead of waiting for a video to end, the SDK calculates a similarity score (0.0 to 1.0) for every single frame as it arrives.
*   **The 1-Frame Algorithmic Delay:** To confirm that an action has reached its completion peak, the NMS logic requires a "falling edge" (the score must drop). Therefore, the `Detector::push_frame()` function will emit a detection event exactly one frame *after* the mathematical peak of the action occurs.

### 4. The Model-Runtime Data Contract (Baked-In Settings)
The SDK is an obedient execution engine; the "brain" is the `.int8` model. During model creation in the Willow Cloud Oracle, specific configuration parameters are permanently locked inside the 24-byte binary header of the model. **You cannot override these in your C++ application.** 

If your application feels too strict, too slow, or is dropping repetitions, you must understand these baked-in rules. The solution is often to adjust these parameters in the Willow Cloud and provision a new model, rather than trying to hack the SDK.

*   **DTW Sensitivity:** This dictates the mathematical strictness of the confidence score. If you are building an application for amateur fitness, but the model you downloaded was baked with a highly strict DTW sensitivity (designed for professional scouting), the SDK will rarely trigger. The SDK isn't broken; the model is simply demanding a level of mechanical perfection your users do not possess.
*   **Overlap Tolerance:** This setting controls the Non-Maximum Suppression logic, which prevents the SDK from triggering multiple times for a single movement. However, if you are tracking rapid, continuous motions (e.g., a boxing combination or a juggling routine), a high overlap tolerance will cause the SDK to artificially delete back-to-back repetitions. You must tune models specifically for isolated events versus rapid-fire events in the Cloud Oracle.
*   **Zone Bitmask:** The model tells the SDK which specific body parts it is allowed to look at. If a model was trained as a "Tabletop" model that only evaluates the hands and arms, the SDK will completely drop all lower-body coordinates from its calculations to save memory and compute. Do not waste time feeding full-body data into a localized model and wondering why leg movements aren't affecting the confidence score.
*   **Tempo Variance:** This dictates the allowed speed limit of the action (Speed Gating). If the original model was built from a one-second baseball pitch, and the tempo variance is set strictly, a user performing a slow-motion drill will be rejected by the SDK. If you want to support variable speeds or slow-motion captures, you need to provision a model baked with a high tempo variance.

---

## Part 2: Domain-Specific Integrations

### Use Case A: Sim-to-Real & Humanoid Robotics
**The Goal:** Translating human video into training data for Reinforcement Learning (RL) agents in NVIDIA Isaac Sim or ROS.
**How to use Willow:** 
1. Pipe raw human point clouds through `Transforms::to_ros()`.
2. Pass the aligned data through `Retargeter::compute_lookat_rotation()`.

**The Engineering Reality Check (Geometric IK Constraints):**
The Willow SDK calculates Quaternions using *Geometric Forward Kinematics*. It constructs 3x3 orthonormal basis matrices by taking the cross-products of bone vectors (e.g., using the Shoulder-Elbow-Wrist triangle to determine the "Roll" of the arm).
*   **The Limitation:** The SDK assumes the input data is physically possible. It does not contain a constrained Inverse Kinematics (IK) solver (like FABRIK). It does not "know" that a human elbow cannot bend backward. 
*   **The Mitigation:** If your pose estimation sensor glitches and outputs a backward-bending arm, the SDK will faithfully output a backward-bending Quaternion. Robotics teams must implement **Euler Clamping** (hard-coding physical joint limits) or pass the SDK’s output through their simulation engine's native IK constraint solver before sending the data to a physical actuator.

### Use Case B: Edge Biomechanics & Sports Coaching
**The Goal:** Providing real-time feedback on movement quality, acceleration, and peak power without cloud latency.
**How to use Willow:** Pass specific joint trajectories (e.g., the wrist) into the `Evaluator` derivative functions (e.g., `Evaluator::calculate_velocity()`).

**The Engineering Reality Check (The Calculus of Sensor Noise):**
The `Evaluator` uses continuous numerical gradients ($d/dt$) to calculate Velocity (1st derivative), Acceleration (2nd derivative), and Jerk (3rd derivative). 
*   **The Limitation:** Standard pose estimation networks (like MediaPipe) inherently produce sub-pixel "jitter" from frame to frame. While this jitter is invisible to the human eye, the 1st derivative amplifies it. The 3rd derivative (Jerk) amplifies it exponentially.
*   **The Mitigation:** The Cloud Oracle heavily filters models during creation, but the Edge SDK evaluates *raw, live data*. If you feed unfiltered webcam data into the `Evaluator`, your Acceleration and Jerk metrics will be heavily polluted by noise. **Integration engineers MUST apply a low-pass filter (e.g., a Kalman filter or Savitzky-Golay filter) to the 3D coordinate stream *before* calling the Willow SDK.** Garbage data in equals garbage physics out.

### Use Case C: Industrial Safety & Ergonomics
**The Goal:** Detecting dangerous lifting mechanics or gait imbalances on warehouse floors.
**How to use Willow:** Deploy the `Detector::push_frame()` engine on edge cameras. When a "Heavy Lift" action is triggered, evaluate the spine angle at that exact timestamp.

**The Engineering Reality Check (Perspective & Scale Collapse):**
*   **The Limitation:** As noted in Part 1, the SDK relies entirely on **Torso-Length Normalization**. Monocular cameras (standard 2D lenses) struggle to infer Z-axis depth. If an industrial camera is mounted on a 20-foot ceiling looking steeply down at a worker, the worker's Torso Length will severely foreshorten in the 2D projection. 
*   **The Result:** If the Torso Length collapses, the normalization scale shrinks, inflating the relative distance of all other movements. The DTW engine will fail to recognize the action because the live RDM no longer matches the pristine model.
*   **The Mitigation:** You cannot out-train a terrible camera angle. The solution is **Hardware Placement and UX**. Cameras must be placed as close to chest-height as possible, minimizing severe pitch angles. If your application relies on mobile phones, you must build UI overlays that guide the user to position their phone correctly before recording.

---

## Part 3: Model Sourcing Strategy (Synthetic vs. Empirical)

When provisioning models via the Willow Cloud for your application, the source of the model (FBX vs. Video) dictates the resilience of the SDK on your edge hardware.

### 1. Synthetic Models (Source: FBX / Motion Capture)
**Definition:** Models created by importing clean animation data (FBX) or professional Motion Capture (MoCap) data.
**Characteristics:** These models are mathematically perfect. There is no occlusion, and the Z-axis depth is absolute ground truth.
**Deployment Target:** Best for **Robotics, VR Headsets, and Multi-Camera Arrays**.
*   **The Caveat:** Because these models are perfect, they are intolerant of the depth distortion artifacts inherent in single 2D cameras. If you use a perfect FBX model on a cheap 2D webcam, the SDK may reject valid user actions because the user's inferred depth map looks "warped" compared to the perfect ground truth.

### 2. Empirical Models (Source: RGB Video)
**Definition:** Models created by uploading a standard 2D video of a human performing the action.
**Characteristics:** These models contain **"Baked-in Bias."** The AI pose estimator used to create the model made specific errors in guessing the depth (Z-axis) based on the camera angle.
**Deployment Target:** Best for **Mobile Phones, Webcams, and Single-Camera Edge Devices**.
*   **The "Bias Matching" Phenomenon:** For single-camera apps, an Empirical Model often outperforms a Synthetic Model. Why? Because the pose estimator on the user's phone will likely make the *exact same depth-guessing errors* that the pose estimator made when creating the model. Because the live errors match the model's baked-in errors, the mathematical distance remains low, and recognition succeeds.

### 3. Summary Recommendation
*   **If you control the hardware** (e.g., a robot with a depth sensor): Use Synthetic (FBX) models for maximum precision.
*   **If you rely on user hardware** (e.g., a consumer mobile app): Use Empirical (Video) models recorded at an angle similar to the expected user setup.

---

## Summary: Building on the Willow Standard

The Willow 5 Runtime is an extraordinarily powerful, ultra-compressed mathematics engine. By distilling massive neural networks into lightweight `.int8` signatures, we allow you to deploy complex spatial intelligence to microcontrollers and headsets that previously lacked the compute power to run them.

However, the SDK is a deterministic engine. It treats the data you give it as the absolute truth. To achieve enterprise-grade reliability:
1. **Filter your sensor streams.**
2. **Control your camera environments.**
3. **Constrain your robotic actuators.**
4. **Match your model source to your deployment hardware.**

If you control the physical inputs, the Willow SDK will flawlessly handle the spatial intelligence.