package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 5;
    public static final int RIGHT_FOLLOWER_ID = 6;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class FuelConstants {
    public static final int FEEDER_MOTOR_ID = 1;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 2;

    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    public static final double INTAKING_FEEDER_VOLTAGE = 12; //intake rear, left trigger, counterclockwise (metal side facing left)
    public static final double INTAKING_INTAKE_VOLTAGE = -10; //intake front, left trigger, counterclockwise (metal side facing left)
    // if the orange wheel is not upto speed decrease this value so that it gets enough time to speed up to throw farther
    public static final double LAUNCHING_FEEDER_VOLTAGE = -9; //launch rear, right trigger, counterclockwise (metal side facing left) , negative pushes ball toward launcher
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = -12; //launch front, right trigger, counterclockwise (metal side facing left)
    public static final double SPIN_UP_FEEDER_VOLTAGE = 6;
    // try to increase the speed so that the orange wheel gets time to fully get upto speed
    public static final double SPIN_UP_SECONDS = 1.5; // Time to run the feeder at spin-up voltage before launching. Negative value means to run indefinitely until the shooter is ready.
  }

  public static final class ShooterConstants {
    public static final int UPPER_FLYWHEEL_ID = 7;
    public static final int LOWER_FLYWHEEL_ID = 8;
    public static final int ACCELERATOR_ID = 9;

    public static final int FLYWHEEL_CURRENT_LIMIT = 40;

    public static final double SHOOTER_kP = 0.0005;
    public static final double SHOOTER_kI = 0.0;
    public static final double SHOOTER_kD = 0.0;
    public static final double SHOOTER_kFF = 0.000175;

    public static final double SHOOTER_TARGET_RPM = 4000.0;
    public static final double ACCELERATOR_TARGET_RPM = 2000.0;
    public static final double SHOOTER_TOLERANCE_RPM = 100.0;

    public static final double FEEDER_LAUNCH_VOLTAGE = 9.0;
  }

  public static final class VisionConstants {
    public static final String FRONT_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";
    public static final String REAR_CAMERA_NAME = "HD Camera";
    public static final String FUEL_TRAY_CAMERA_NAME = "HD Camera (1)";

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

    // 2025 Reefscape AprilTag IDs
    public static final int[] REEF_TAG_IDS_RED = {6, 7, 8, 9, 10, 11};
    public static final int[] REEF_TAG_IDS_BLUE = {17, 18, 19, 20, 21, 22};
    public static final int[] PROCESSOR_TAG_IDS = {3, 16};
    public static final int[] BARGE_TAG_IDS = {4, 5, 14, 15};

    public static final double REEF_SCORING_DISTANCE = 0.5;
    public static final double PROCESSOR_SCORING_DISTANCE = 0.3;
    public static final double BARGE_SCORING_DISTANCE = 0.8;
  }

  public static final class AutoConstants {
    public static final double MAX_AUTO_SPEED = 0.9;
    public static final double AUTO_DRIVE_KP = 1.5;
    public static final double AUTO_ROTATION_KP = 2.0;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DRIVE_SCALING = .9;
    public static final double ROTATION_SCALING = .9;
    public static final double DRIVE_TURBO_SCALING = .9;
    public static final double ROTATION_TURBO_SCALING = .8;

    public static final double DEADBAND = 0.05;  // Exponent for rotation input squaring
    public static final double ROTATION_SENSITIVITY = 2.0;// Deadband for joystick inputs. This helps prevent the robot from moving when the joystick is near the center position.
    public static final double DRIVE_SENSITIVITY = 2.0; // Exponent for drive input squaring. Higher values give finer control at low speeds.
  }
}
