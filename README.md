#UR3e Autonomous Tic-Tac-Toe with Closed-Loop Vision

A fully autonomous Tic-Tac-Toe playing robot built in MATLAB using a UR3e collaborative robot arm and overhead camera vision. This project features dynamic AprilTag tracking, Augmented Reality (AR) debugging, and closed-loop error recovery, allowing the robot to play a flawless physical game against a human opponent.
✨ Key Features

    Dual-Tag Sensor Fusion: Utilizes two origin AprilTags (IDs 450 and 460) to establish the board's 3D coordinate frame. If one tag is occluded by a hand or the robot arm, the system dynamically calculates the origin from the secondary tag. If both are visible, it averages their translations to eliminate camera jitter and verifies depth scaling.

    Live AR Vision Radar: Projects the 3D mathematical coordinate frame directly onto the 2D live video feed. It draws 63x63mm green bounding boxes around the physical squares and color-codes all tracked pieces (Red for CPU, Cyan for Player, Magenta for Origin) in real-time for zero-guesswork debugging.

    Closed-Loop Move Verification: After executing a pick-and-place operation, the arm retreats and takes a fresh snapshot of the board. It verifies that the intended piece successfully landed in the target square.

    Dynamic Recovery Protocol: If the verification scan fails (e.g., the piece bounced out of the square or was dropped), the robot does not blindly grab a new piece. It scans the camera feed, calculates the exact 3D pose and rotation of the misplaced piece, picks it up from wherever it landed, and attempts the placement again.

    Right-Angle Path Planning: Calculates perfectly flat, horizontal trajectories (matching Z-heights between hover poses) to ensure the arm never swoops diagonally and knocks over existing pieces.

🛠️ Hardware Requirements

    Robot: Universal Robots UR3e Cobot

    Camera: DFK 23U618 Overhead Camera

    Board: 9-square grid (63mm x 63mm squares)

    AprilTags (tag36h11):

        Board Origin Tags: 450, 460 (Placed exactly 150mm apart on the X-axis)

        CPU Pieces: 461, 462, 463, 464, 465 (25mm)

        Player Pieces: 451, 452, 453, 454, 455 (25mm)

💻 Software Requirements

    MATLAB

    Computer Vision Toolbox

    Robotics System Toolbox

    URQt Toolbox (Custom UR interface)

📂 Repository Structure
The Runtime Files (Required for Gameplay)

    main_TicTacToe.m - The main game loop containing the vision pipeline, move logic, AR radar, and robot control.

    HandheldCal/HandCalInfo.mat - Saved intrinsic camera parameters (lens distortion matrix).

    RobCamCal/URcoCalInfo.mat - Hand-Eye calibration matrix (Hc2o​) mapping the camera frame to the robot's base frame.

    RobCamCal/SafeRestPose.mat - Custom 6-joint configuration to park the arm safely out of the camera's field of view.

The Calibration Scripts (Archived)

These scripts were used to generate the .mat files above but are not needed for runtime.

    CamParamGet.m - Processes checkerboard images to solve intrinsic parameters.

    teachCalibrationPoses.m - Records 15 joint configurations for hand-eye calibration.

    AUTOCAMCAL.m - Computes the transformation matrix between the UR3e and the camera.

    teachSafeHome.m - Saves a custom joint configuration to a file.

🚀 How to Play

    Ensure the UR3e is powered on and in Remote Control mode.

    Place the board under the camera so both origin tags (450 and 460) are visible.

    Place the 5 CPU pieces in a staging area visible to the camera.

    Run main_TicTacToe.m in MATLAB.

    The Live AR Radar will appear. Verify the green bounding boxes perfectly encapsulate the physical board squares.

    Enter 1 (CPU First) or 2 (Player First) in the Command Window.

    Play! The camera will automatically detect when you have placed a piece and removed your hand.
