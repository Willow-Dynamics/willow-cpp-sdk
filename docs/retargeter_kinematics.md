# Retargeter: Kinematics

The `Retargeter` module maps the absolute 3D positional data (`Point3D`) outputted by computer vision systems into hierarchical joint rotations (`Quaternions`). 

While the `Detector` tells you **what** action is happening, the `Retargeter` provides the data needed to **replay** that action on a 3D Avatar (Unity/Unreal) or physical hardware (Robotics servo chains).

## 1. The Look-At Computation
The core mechanism used by the Willow Engine is the "Look-At" rotation. It calculates the specific orientation (Quaternion) required for a parent joint (e.g., Shoulder) to point its bone vector directly at a child joint (e.g., Elbow).

## 2. Implementation
To compute a rotation, you must provide:
1. **Position:** The 3D world position of the parent joint.
2. **Target:** The 3D world position of the child joint.
3. **Up Reference:** A normalized vector defining the "Up" direction for the world (usually `{0, 1, 0}`), which prevents the joint from rolling unpredictably.

```cpp
// Example: Computing the rotation for a Right Shoulder joint
willow::Point3D shoulder_pos = {0.5f, 1.5f, 0.0f, 1.0f};
willow::Point3D elbow_pos    = {0.8f, 1.2f, 0.2f, 1.0f};
willow::Point3D world_up     = {0.0f, 1.0f, 0.0f, 1.0f};

// Compute the Quaternion (w, x, y, z)
willow::Quaternion rot = willow::Retargeter::compute_lookat_rotation(
    shoulder_pos, 
    elbow_pos, 
    world_up
);

std::cout << "Rotation: " << rot.w << ", " << rot.x << ", " << rot.y << ", " << rot.z << "\n";
```

### Applying to 3D Rigs
This Quaternion can be applied directly to the **Local Rotation** transform component of a bone in Unity, Unreal Engine, or a ROS TF tree.