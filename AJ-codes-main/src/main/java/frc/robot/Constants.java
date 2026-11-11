package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 5;
    public static final int RIGHT_FOLLOWER_ID = 6;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
  }

  public static final class FuelConstants {
    public static final int FEEDER_MOTOR_ID = 1;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 2;

    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 30;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 30;

    // PID + Feedforward for closed-loop RPM control
    // FF is roughly 1 / NEO free speed (5676 RPM) — gets you close, P handles the rest
    // Tune these on the real robot: if RPM oscillates, lower kP. If it never reaches target, raise kFF.
    public static final double FUEL_kP = 0.000150;
    public static final double FUEL_kI = 0.0;
    public static final double FUEL_kD = 0.0;
    public static final double FUEL_kFF = 0.00250; 

    // Closed-loop voltage ramp for launcher/feeder — kills 0→Nrpm setpoint-jump oscillation.
    // 0.25s means full voltage swing reached in 250 ms; tune down if shots feel sluggish.
    public static final double FUEL_RAMP_RATE_SECONDS = 0.25;

    // All speeds in RPM — tune on real robot
    public static final double INTAKING_FEEDER_RPM = 2500;     // intake rear, left trigger
    public static final double INTAKING_INTAKE_RPM = -2500;    // intake front, left trigger
    public static final double LAUNCHING_FEEDER_RPM = -2700;   // launch rear, negative pushes ball toward launcher
    public static final double LAUNCHING_LAUNCHER_RPM = -1200; // teleop right-trigger shot
    public static final double AUTO_LAUNCHING_LAUNCHER_RPM = -2700; // auto 15s shoot — softer than teleop
    public static final double SPIN_UP_FEEDER_RPM = 500;  // holds ball back while launcher spins up
    public static final double SPIN_UP_SECONDS = 2;
    public static final double UNSTUCK_FEEDER_RPM = 1000;  // Y button reverse-feed speed (negated at callsite)


    // ── Distance-based RPM lookup table ─────────────────────────────────────
    // Maps distance (meters) → launcher RPM — reserved for future vision-based shooting.
    // Not currently used by auto (fixed AUTO_LAUNCHING_LAUNCHER_RPM) or teleop (fixed LAUNCHING_LAUNCHER_RPM).
    public static final InterpolatingDoubleTreeMap LAUNCHER_RPM_TABLE = new InterpolatingDoubleTreeMap();
    static {
      LAUNCHER_RPM_TABLE.put(0.5,  -2300.0);
      LAUNCHER_RPM_TABLE.put(1.0,  -2350.0);
      LAUNCHER_RPM_TABLE.put(1.5,  -2450.0);
      LAUNCHER_RPM_TABLE.put(2.0,  -2550.0);
      LAUNCHER_RPM_TABLE.put(2.5,  -2700.0);
      LAUNCHER_RPM_TABLE.put(3.0,  -2900.0);
    }
  }

  public static final class VisionConstants {
    public static final String FRONT_CAMERA_NAME = "APRILFRONT";
    public static final String REAR_CAMERA_NAME = "APRILBACK";
    public static final String FUEL_TRAY_CAMERA_NAME = "INSIDE";

    // PhotonVision MJPEG stream URLs — shown in SmartDashboard as camera widgets
    // If streams don't appear, check port numbers in PhotonVision dashboard settings
    public static final String FRONT_CAMERA_STREAM_URL = "http://photonvision.local:1181/stream.mjpg";
    public static final String REAR_CAMERA_STREAM_URL = "http://photonvision.local:1182/stream.mjpg";
    public static final String FUEL_TRAY_STREAM_URL = "http://photonvision.local:1183/stream.mjpg";

    public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(0.3, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-15), 0));

    public static final Transform3d REAR_CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(-0.3, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-15), Math.PI));

    public static final double AIM_KP = 0.03;
    public static final double AIM_TOLERANCE_DEGREES = 2.0;

    public static final double RANGE_KP = 0.3;
    public static final double RANGE_TOLERANCE_METERS = 0.05;

    public static final double MAX_POSE_AMBIGUITY = 0.2;
    public static final double MAX_POSE_DISTANCE_METERS = 4.0;

    public static final double JAM_DETECTION_TARGET_AREA_THRESHOLD = 5.0;
    public static final int JAM_FRAME_THRESHOLD = 25;
    public static final double JAM_AREA_CHANGE_THRESHOLD = 0.5;

    // 2026 REBUILT AprilTag IDs — HUB scoring targets (same for both alliances)
    public static final int[] HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};

    public static final double HUB_SCORING_DISTANCE = 0.5;
    // Primary Hub tag to aim at — picked automatically based on alliance
    // Red: Tag 4 faces the most common starting positions on red side
    // Blue: Tag 25 faces the most common starting positions on blue side
    public static final int AUTO_TARGET_TAG_RED = 4;
    public static final int AUTO_TARGET_TAG_BLUE = 25;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DRIVE_SCALING = 0.5;
    public static final double ROTATION_SCALING = 0.6;
    public static final double DRIVE_SLOW_SCALING = 0.6;
    public static final double ROTATION_SLOW_SCALING = 0.5;

    public static final double DEADBAND = 0.03;  // Deadband for joystick inputs — prevents drift near center.
    public static final double ROTATION_SENSITIVITY = 2;   // Exponent for rotation input curve — higher = finer control at low stick deflection.
    public static final double DRIVE_SENSITIVITY = 2.5;    // Exponent for drive input curve — higher = finer control at low stick deflection.

    // Slew rate limits for teleop Drive command (units per second)
    public static final double DRIVE_SLEW_PER_SEC    = 3.0;  // 0→full forward in ~333 ms
    public static final double ROTATION_SLEW_PER_SEC = 5.0;  // snappier turning

    // NOTE: Drive.java applies deadband+scaling FIRST, then the exponent curve.
    // Effective peak forward output is ≈ DRIVE_SCALING^DRIVE_SENSITIVITY ≈ 0.9^2.5 ≈ 0.77.
    // Intentional — drivers have muscle memory on this feel. Do not change mid-season.
  }
}
