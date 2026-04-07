# Topology and Transforms

## 1. The 75-Point Skeleton Map
The Willow Engine relies on a fixed 75-point hybrid skeleton array. It is the developer's responsibility to map raw tracking data (e.g., MediaPipe, Azure Kinect, Quest 3) into this structure before passing it to the `Detector`.

The indices are strictly mapped as follows:

### Body (MediaPipe Pose Standard)
* **0:** Nose
* **1:** Left Eye (Inner)
* **2:** Left Eye
* **3:** Left Eye (Outer)
* **4:** Right Eye (Inner)
* **5:** Right Eye
* **6:** Right Eye (Outer)
* **7:** Left Ear
* **8:** Right Ear
* **9:** Mouth (Left)
* **10:** Mouth (Right)
* **11:** Left Shoulder
* **12:** Right Shoulder
* **13:** Left Elbow
* **14:** Right Elbow
* **15:** Left Wrist
* **16:** Right Wrist
* **17:** Left Pinky (Knuckle)
* **18:** Right Pinky (Knuckle)
* **19:** Left Index (Knuckle)
* **20:** Right Index (Knuckle)
* **21:** Left Thumb (Knuckle)
* **22:** Right Thumb (Knuckle)
* **23:** Left Hip
* **24:** Right Hip
* **25:** Left Knee
* **26:** Right Knee
* **27:** Left Ankle
* **28:** Right Ankle
* **29:** Left Heel
* **30:** Right Heel
* **31:** Left Foot Index
* **32:** Right Foot Index
* **75:** Tracked Object **(Physics Mode Only)** The metric XYZ centroid of the physical object interacting with the human (e.g. Ball, Tool, Chassis).

### Left Hand (MediaPipe Hand Standard)
* **33:** Wrist
* **34:** Thumb CMC
* **35:** Thumb MCP
* **36:** Thumb IP
* **37:** Thumb Tip
* **38:** Index Finger MCP
* **39:** Index Finger PIP
* **40:** Index Finger DIP
* **41:** Index Finger Tip
* **42:** Middle Finger MCP
* **43:** Middle Finger PIP
* **44:** Middle Finger DIP
* **45:** Middle Finger Tip
* **46:** Ring Finger MCP
* **47:** Ring Finger PIP
* **48:** Ring Finger DIP
* **49:** Ring Finger Tip
* **50:** Pinky MCP
* **51:** Pinky PIP
* **52:** Pinky DIP
* **53:** Pinky Tip

### Right Hand (MediaPipe Hand Standard)
* **54:** Wrist
* **55:** Thumb CMC
* **56:** Thumb MCP
* **57:** Thumb IP
* **58:** Thumb Tip
* **59:** Index Finger MCP
* **60:** Index Finger PIP
* **61:** Index Finger DIP
* **62:** Index Finger Tip
* **63:** Middle Finger MCP
* **64:** Middle Finger PIP
* **65:** Middle Finger DIP
* **66:** Middle Finger Tip
* **67:** Ring Finger MCP
* **68:** Ring Finger PIP
* **69:** Ring Finger DIP
* **70:** Ring Finger Tip
* **71:** Pinky MCP
* **72:** Pinky PIP
* **73:** Pinky DIP
* **74:** Pinky Tip

### Handling Missing Data
If your tracking system does not track hands (or feet), you **must** still provide the full 75-point vector to maintain array alignment. 

Crucially, the Willow Engine utilizes a 4-dimensional Point structure: `{X, Y, Z, Visibility}`. If a joint is untracked or occluded, you must set its **Visibility to `0.0f`**.

Simply leave the untracked indices (e.g., 33-74) as zero-initialized points: 
`{0.0f, 0.0f, 0.0f, 0.0f}`.

The `zone_mask` and internal visibility thresholds will instruct the engine to safely ignore these low-confidence indices during the distance calculation, preventing them from corrupting the geometric signature.

---

## 2. Coordinate Bridges (Transforms)
The Willow Cloud Oracle mathematically calibrates all models in **MediaPipe Screen Space**. This is the native coordinate system of the engine logic:
* **+X:** Right
* **+Y:** Down (Standard screen coordinates)
* **+Z:** Forward (Depth into the screen)

If you are rendering the skeleton for visualization or acting on the results in a different physics engine, you must use the `Transforms` module to map the data correctly.

### Unity Bridge (Left-Handed)
Unity uses a **Y-Up, Left-Handed** coordinate system. The transform inverts the Y-axis to map screen-space to world-space.

```cpp
// Assuming raw_skel is a std::vector of your native tracker's points
willow::Skeleton raw_skel = get_tracking_data();

// Initialize the 75-point Willow array
willow::Skeleton unity_skel(75);

for(size_t i=0; i<75; ++i) {
    // Maps (+X, +Y, +Z) -> (+X, -Y, +Z)
    unity_skel[i] = willow::Transforms::to_unity(raw_skel[i]); 
}
```

### ROS / Robotics Bridge (Right-Handed)
The Robotics Operating System (ROS) generally relies on **Z-Up, X-Forward (Right-Handed)**. The transform swaps axes to map "Screen Space" to "Real-World Ground Space".

```cpp
// Assuming raw_skel is a std::vector of your native tracker's points
willow::Skeleton raw_skel = get_tracking_data();
willow::Skeleton robot_skel(75);

for(size_t i=0; i<75; ++i) {
    // Maps (+X, +Y, +Z) -> (+Z, -X, -Y)
    // Depth becomes Height (Z)
    // Right becomes Side (-Y)
    // Down becomes Back (-X)
    robot_skel[i] = willow::Transforms::to_ros(raw_skel[i]);     
}
```