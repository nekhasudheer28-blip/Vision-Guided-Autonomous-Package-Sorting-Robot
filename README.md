# Vision-Guided Autonomous Package Sorting Robot 

## Project Overview
This is a capstone project for B.Tech Mechatronics and Automation at VIT Chennai. The system is an autonomous package sorting robot for small-scale warehouse operations. It uses a 4-DOF robotic arm (3 revolute joints + servo gripper), a stationary overhead RGB camera, a checkerboard calibration surface, and computer vision to autonomously detect, classify, pick, and sort packages into designated zones — all without human intervention.

The user issues a text prompt (e.g., "sort fragile") and the robot scans the workspace, classifies all packages by their color-coded stickers, picks only the matching ones (nearest-first), and places them in the correct drop zone. The entire software pipeline runs on ROS2 Humble.

---

## Hardware

| Component | Specification |
|---|---|
| Robotic arm | 4-DOF: 3 revolute joints (MG995 servos) + 1 gripper (SG90 servo), 3D-printed PLA links |
| Link lengths | L1 = 13.5 cm (base to shoulder), L2 = 12.0 cm (shoulder to elbow), L3 = 24.0 cm (elbow to gripper tip) |
| Microcontroller | Arduino Uno (ATmega328P), 9600 baud serial to host PC |
| Camera | USB RGB desktop camera, 1280×720, mounted on a fixed stand opposite the robot |
| Calibration board | 9×6 inner corners checkerboard, 2.9 cm square size, placed between robot and camera |
| Packages | Small cardboard boxes (~3 cm tall) with color-coded stickers on top face |
| Power | External 5V 5A adapter for servos, laptop USB for Arduino |

---

## Kinematic Configuration

The arm is an RRR (Revolute-Revolute-Revolute) serial manipulator:
- **Joint 1 (Base):** Rotates about the vertical Z-axis (yaw). Servo on pin 2.
- **Joint 2 (Shoulder):** Pitch rotation in the vertical plane. Servo on pin 3.
- **Joint 3 (Elbow):** Pitch rotation in the vertical plane. Servo on pin 4.
- **Gripper:** Open/close only. Servo on pin 5. Open = 90°, Closed = 10°.

### Forward Kinematics (DH Convention)

DH parameter table:

| Link | θ (variable) | d (cm) | a (cm) | α |
|---|---|---|---|---|
| 1 | θ₁ | 13.5 | 0 | 90° |
| 2 | θ₂ | 0 | 12.0 | 0° |
| 3 | θ₃ | 0 | 24.0 | 0° |

End-effector position from FK:
```
x = cos(θ₁) · (L₂·cos(θ₂) + L₃·cos(θ₂+θ₃))
y = sin(θ₁) · (L₂·cos(θ₂) + L₃·cos(θ₂+θ₃))
z = L₁ + L₂·sin(θ₂) + L₃·sin(θ₂+θ₃)
```

### Inverse Kinematics (Closed-Form Analytical)

Solved geometrically — no iterative solver needed:

1. **Base angle:** θ₁ = atan2(y, x)
2. **Planar decomposition:** r = √(x²+y²), z' = z − L₁
3. **Elbow angle (law of cosines):** cos(θ₃) = (r²+z'²−L₂²−L₃²)/(2·L₂·L₃), then θ₃ = atan2(±√(1−cos²θ₃), cos θ₃). Elbow-up configuration exclusively used.
4. **Shoulder angle:** k₁ = L₂+L₃·cos(θ₃), k₂ = L₃·sin(θ₃), θ₂ = atan2(r,z') − atan2(k₂,k₁)

Joint limits (derived from workspace analysis with safety margin):
- θ₁: −55° to +55°
- θ₂: −50° to +85°
- θ₃: +40° to +148°

### Servo Offset Mapping

Kinematic angles → servo pulse angles:
```
servo1 = 90.00 + θ₁
servo2 = 115.71 − θ₂    (reversed direction)
servo3 = 6.69 + θ₃
```
These offsets were measured empirically at kinematic zero pose. Servo range clamped to 5°–175°.

### IK Validation Results
- 50 targets tested, 100% success rate
- Average FK verification error: 0.18 cm (max 0.47 cm, tolerance ±0.5 cm)
- Solver execution time: 0.23 ms per query

---

## Camera Calibration

Two-phase calibration process (inspired by Kai Nakamura's RBE3001 project at WPI):

### Phase 1: Intrinsic Calibration
- Checkerboard held at ~20 different positions/tilts/distances in front of camera
- OpenCV `findChessboardCorners` + `cornerSubPix` for sub-pixel corner detection
- `calibrateCamera` computes 3×3 intrinsic matrix K (focal lengths fx, fy + principal point cx, cy) and distortion coefficients
- Reprojection RMS error achieved: 1.43 pixels

### Phase 2: Extrinsic Calibration
- Board placed in fixed working position on table
- Single undistorted frame captured
- OpenCV `solvePnP` (iterative method) computes rotation matrix R and translation vector t (camera pose relative to board)
- Camera height automatically extracted: 40.20 cm above board
- No manual height measurement needed

### Coordinate Transformation Pipeline

```
Pixel (u,v) → Homography H⁻¹ → Board coords (cm) → T_checker_to_robot → Robot coords (x,y,z)
```

- **Homography H:** Built from intrinsic matrix and extrinsic params: H = K_new · [r1 | r2 | t]
- **T_checker_to_robot:** 4×4 homogeneous transform encoding physical offset from checkerboard origin to robot base origin. Measured with ruler: dx = 19.0 cm (board to robot's right), dy = 8.7 cm (board forward of robot base). Rotation matrix accounts for axis alignment between board frame and robot's IK convention (θ₁ = atan2(y,x)).
- **Projection height correction:** Corrects for parallax since boxes sit above the checkerboard plane (3 cm box height at 40.2 cm camera height → ~2-4 mm correction using similar triangles).

### Coordinate Accuracy
- Mean spatial error: 3.2 mm
- Maximum error: 7.1 mm
- Within gripper jaw tolerance (~15 mm)

---

## Object Detection

### Box Detection (Color-Agnostic)
Uses checkerboard pattern interruption + edge detection instead of HSV color thresholding. This makes detection robust to varying box colors, stickers, and the wooden table surface.

Pipeline:
1. Capture background frame (empty board) at startup
2. Each new frame: subtract background to find regions of change
3. Brightness equalization (normalize V channel in HSV)
4. Undistort frame using calibration params
5. HSV color mask for cardboard hue (H: 8-25, S: 50-175, V: 60-220)
6. Morphological operations (open + close) to clean mask
7. Contour detection → filter by area and aspect ratio
8. Centroid of each contour → pixel coordinates
9. Apply homography + T_checker_to_robot → robot frame coordinates (x, y, z=1.5 cm)

### Package Classification (Sticker-Based)
Five categories identified by sticker color on box top face:

| Category | Sticker Color | HSV Hue Range |
|---|---|---|
| Fragile | Red | H: 0-10 or 160-179 |
| Zone A | Blue | H: 100-130 |
| Zone B | Yellow | H: 20-35 |
| Immediate Delivery | Green | H: 40-80 |
| Damaged | Orange | H: 10-20 |

Classification method (no ML needed for stickers):
1. Crop bounding box of each detected box from undistorted frame
2. Subtract corresponding background region → isolates sticker pixels
3. Convert to HSV, compute dominant hue from H-channel histogram
4. Map hue to nearest category

### ML Model (YOLOv8 — Built Separately by Team Member)
- Trained on 4222 images of cardboard boxes
- Classes: standard boxes, carton boxes, cracked cartons, opened cartons, wet cartons
- Used for detecting box types and damage on larger boxes
- Includes OCR stage for reading text labels/stickers
- Demonstrated separately during presentation; integration with the sorting pipeline is future work

### Vision Performance
- YOLOv8 mAP @ IoU 0.5: 91.4%
- False positive rate: <3%
- Inference time: 112 ms/frame (~9 fps) on CPU
- Pipeline runs at 5 Hz

---

## Pick and Place Execution

### Serial Protocol (Python → Arduino)
```
"T1,T2,T3\n"        → move at normal speed, gripper unchanged
"T1,T2,T3,G\n"      → move at normal speed + set gripper angle
"T1,T2,T3,G,1\n"    → move at SLOW speed + set gripper angle
```
Arduino responds: `"OK:s1,s2,s3,g\n"` or `"ERR:reason\n"`

### Motion Profiles
| Profile | Step Size | Delay | Use |
|---|---|---|---|
| Normal | 1.5°/step | 12 ms | Fast transit (approach, retract) |
| Slow | 0.5°/step | 25 ms | Gentle descent, grasping, lifting |

All 3 arm servos sweep simultaneously using an interleaved stepping algorithm (`sweepAll()`). Gripper always moves at slow speed.

### Pick and Place Sequence (Per Box)
1. Move to HOME (out of camera FOV)
2. Camera scans and classifies ALL boxes
3. Filter by prompt (e.g., "fragile"), sort nearest-first by √(x²+y²)
4. For each matching box:
   - Move to APPROACH (x, y, z=6cm) — normal speed
   - Descend to GRASP (x, y, z=1.5cm) — slow speed
   - Close gripper — slow
   - Lift to APPROACH — slow
   - Transit to DROP ZONE — normal speed
   - Descend to PLACE — slow
   - Open gripper — slow
   - Lift and return HOME
5. Repeat for next matching box

### Performance
- 20 packages tested, 18/20 successful (90% success rate)
- Average cycle time: 7.8 seconds
- Throughput: 7.7 picks/minute
- Fastest: 6.4 sec (near base), Slowest: 10.2 sec (far edge)
- 2 failures due to gripper backlash at open/closed transition

---

## ROS2 Software Architecture

Built on ROS2 Humble with 7 nodes:

| Node | Function |
|---|---|
| `package_log_bridge` | Reads detection JSON → publishes PoseArray to `/detected_boxes_camera` |
| `camera_to_base_node` | TF2 transform: camera frame → robot base frame → publishes to `/detected_boxes_base` |
| `box_pick_sequencer` | Finite state machine (approach→descend→grip→lift→move→place→release→retreat), sorts leftmost-first |
| `pick_target_ik_driver` | Reads URDF, computes FK/IK, publishes JointTrajectory |
| `arm_ik_controller` | Receives target poses, runs IK solver, sends serial commands to Arduino |
| `robot_state_publisher` | Publishes TF2 tree from URDF |
| `static_transform_publisher` | Camera frame offset |

All launched via single launch file: `full_system.launch.py`

URDF model used for RViz visualization of arm in real-time.

---

## Dashboard (UI/UX)

Web-based dashboard showing:
- Live camera feed
- Real-time inventory tracking (box picked → inventory updated)
- Box classification results and counts
- Robot arm status and current joint angles

---

## File Structure

```
project/
├── main.py                    ← Master orchestrator (prompt interface + sorting loop)
├── detect_boxes.py            ← Color-agnostic box detection
├── classify_box.py            ← Sticker color classification
├── robot_arm_IK.py            ← Python IK solver
├── calibrate.py               ← Two-phase camera calibration
├── test_coordinate.py         ← Interactive coordinate testing tool
├── pick_and_place.py          ← Full pick-and-place sequence
├── calibration.npz            ← Saved calibration data (K, dist, H, T_checker_to_robot)
├── robot_arm_pick_place.ino   ← Arduino firmware (servo control + serial protocol)
└── ros2_workspace/
    ├── launch/
    │   └── full_system.launch.py
    ├── config/
    │   └── vision_pipeline.yaml
    └── src/
        ├── package_log_bridge.py
        ├── camera_to_base_node.py
        ├── box_pick_sequencer.py
        ├── pick_target_ik_driver.py
        ├── arm_ik_controller.py
        └── arm.urdf
```

---

## Workspace

- Effective area: x ∈ [10, 29] cm, y ∈ [−11.7, +11.7] cm (relative to robot base)
- Grasp height: z = 1.5 cm (half of 3 cm box height)
- Approach height: z = 6 cm (clearance above box)
- Checkerboard: 9×6 inner corners, 2.9 cm squares

---

## Key Technical Challenges Solved

1. **Coordinate axis alignment:** Camera axes didn't match robot IK convention. Required careful construction of rotation matrix in T_checker_to_robot and systematic verification with ruler measurements at known positions.

2. **HSV detection instability:** HSV thresholding for box detection was unreliable due to lighting variation and wooden table background. Replaced with geometry-based edge detection using checkerboard pattern interruption — made detector fully color-agnostic.

3. **Joint limit tuning:** Initial limits (±85° for θ₃) were too conservative, causing IK failures at valid workspace positions. Systematic workspace analysis at all corner points + servo hardware verification expanded limits safely.

4. **Serial/OpenCV blocking conflict:** cv2.waitKey() polling and Python input() fought for control. Fixed by separating camera display phase from command input phase.

5. **TF2 timing in ROS2:** camera_to_base_node failed with "transform not yet available" at startup. Fixed with retry loop + launch dependency ordering.

6. **Servo horn offset calibration:** Physical servo zero ≠ kinematic zero. Each servo's offset measured empirically with protractor and hardcoded as correction constants.

---

## Cost
Total hardware: ~₹4,500 (vs ₹1,50,000–5,00,000 for commercial systems)

## Future Work
- Integrate ML model (YOLOv8) for OCR-based label reading on actual packages
- Depth camera (Intel RealSense) for 3D localization
- Compliant force-sensing gripper for higher grasp reliability
- Conveyor belt integration for continuous sorting
- Full ROS2 pipeline integration (currently partially integrated)
- UI/UX dashboard completion and testing

---

## Team
- Nikhil Mohan (22BMH1014)
- Nekha Sudheer (22BMH1129)
- Abhinav Shaji Kumar (22BMH1130)

Guide: Dr. Raghukiran Nadimpalli, School of Mechanical Engineering, VIT Chennai

## Reference Project
Kai Nakamura, RBE3001 Unified Robotics III, Worcester Polytechnic Institute
- Report: https://kainakamura.com/rbe3001/report.pdf
- GitHub: https://github.com/KaiNakamura/RBE3001
