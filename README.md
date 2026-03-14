# **Forward & Inverse Kinematics — Unity**
C# implementations of forward and inverse kinematics for a hierarchical skeletal animation system, built within a Unity course framework at SFU.

# **What I Wrote**

- ForwardKinematics.cs: Full FK implementation traversing a joint hierarchy, converting per-joint Euler angles to quaternions respecting all 6 rotation orders (XYZ, XZY, YXZ, YZX, ZXY, ZYX), then computing global positions and orientations via parent-to-child quaternion chaining
- InverseKinematics.cs: CCD (Cyclic Coordinate Descent) IK solver with 100-iteration convergence loop and 0.03 unit threshold. Each iteration rotates joints along the chain toward the target using the cross product axis and dot product angle between the end-effector and target vectors
- Foot penetration detection function: Per-frame stair collision check using stair AABB bounds, correcting foot targets when feet clip through geometry
- Foot sliding correction function: Detects when feet are planted (below height threshold) and locks their position across frames to eliminate sliding artifacts
- Skeleton rendering function: Positions joint spheres and orients bone transforms by computing midpoint and direction between parent/child joint pairs

# **Implementation Notes**

- Rotation order is respected per-joint since BVH motion capture data encodes rotations in varying orders
- The IK chain extends from the end effector all the way up to the root hip joint
- FK is re-run after every CCD iteration to keep global positions consistent with updated local quaternions



**NOTE**: This repo will not run on its own as its missing the required unity scene to run it. This repo is just here to display what I have built and my knowledge of IK and FK systems. 
