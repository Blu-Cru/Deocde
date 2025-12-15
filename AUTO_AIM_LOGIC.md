# 🎯 Auto-Aim Logic: "Map-Based" Targeting

This document explains how the robot aims at the High Basket without looking directly at it.

## 🧠 The Concept: Map-Based Targeting

Unlike older robots that used a camera to "find" the goal (Vision Targeting), this system uses **Localization**.

1.  **Where am I?** The **Limelight** (watching AprilTags) + **Pinpoint** (odometry) tells the robot its exact $(X, Y, Heading)$ on the field.
2.  **Where is the Goal?** We know the High Basket is always at specific coordinates:
    *   **Blue Basket:** $(-72, 72)$
    *   **Red Basket:** $(-72, -72)$
3.  **Aiming:** The robot simply calculates the angle between "My Pos" and "Goal Pos" and turns the turret/shooter to match.

### ✅ Advantages
*   **Shoot Blind:** You can aim even if another robot is blocking your view of the basket.
*   **Shoot While Moving:** The angle updates instantly as you drive.
*   **Faster:** No need to "search" for the target; it's always math.

---

## ⚙️ The Code Flow

### 1. `Robot.java`
*   `getPose()`: Returns the fused Robot Position (Limelight + Odometry).

### 2. `Globals.java`
*   Defines the fixed coordinates of the goals for the "Into the Deep" season.

### 3. `Turret.java`
*   `lockOnGoal()`:
    1.  Calculates vector from Robot $\rightarrow$ Goal.
    2.  Converts to Field Centric Angle.
    3.  Subtracts Robot Heading to get Turret Angle.
    4.  PID Controller moves turret to that angle.

### 4. `Shooter.java`
*   `autoAim()`:
    1.  Calculates Distance (hypotenuse) to goal.
    2.  Uses `ShooterAutoAimInterpolation` to look up the perfect RPM and Hood Angle for that distance.

---

## 🎮 Driver Station Guide

*   **To Test**: Run `LimelightFusionTest` OpMode.
*   **Button A:** Locks Turret to Goal (keep held or toggle).
*   **Button B:** Revs Shooter & Adjusts Hood for current distance.
*   **Joystick:** Drive freely; the turret will keep pointing at the goal!
