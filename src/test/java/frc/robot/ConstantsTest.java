package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

class ConstantsTest {

  @Test
  void allCanIdsAreUnique() {
    Set<Integer> ids = new HashSet<>();
    int[] allIds = {
      Constants.FuelConstants.FEEDER_MOTOR_ID,
      Constants.FuelConstants.INTAKE_LAUNCHER_MOTOR_ID,
      Constants.DriveConstants.LEFT_LEADER_ID,
      Constants.DriveConstants.LEFT_FOLLOWER_ID,
      Constants.DriveConstants.RIGHT_LEADER_ID,
      Constants.DriveConstants.RIGHT_FOLLOWER_ID,
      Constants.ShooterConstants.UPPER_FLYWHEEL_ID,
      Constants.ShooterConstants.LOWER_FLYWHEEL_ID,
      Constants.ShooterConstants.ACCELERATOR_ID
    };

    for (int id : allIds) {
      assertTrue(ids.add(id), "Duplicate CAN ID found: " + id);
    }

    assertEquals(9, ids.size());
  }

  @Test
  void canIdsAreInValidRange() {
    int[] allIds = {
      Constants.FuelConstants.FEEDER_MOTOR_ID,
      Constants.FuelConstants.INTAKE_LAUNCHER_MOTOR_ID,
      Constants.DriveConstants.LEFT_LEADER_ID,
      Constants.DriveConstants.LEFT_FOLLOWER_ID,
      Constants.DriveConstants.RIGHT_LEADER_ID,
      Constants.DriveConstants.RIGHT_FOLLOWER_ID,
      Constants.ShooterConstants.UPPER_FLYWHEEL_ID,
      Constants.ShooterConstants.LOWER_FLYWHEEL_ID,
      Constants.ShooterConstants.ACCELERATOR_ID
    };

    for (int id : allIds) {
      assertTrue(id >= 1 && id <= 62, "CAN ID out of valid range (1-62): " + id);
    }
  }

  @Test
  void shooterPidConstantsAreNonNegative() {
    assertTrue(Constants.ShooterConstants.SHOOTER_kP >= 0, "kP must be non-negative");
    assertTrue(Constants.ShooterConstants.SHOOTER_kI >= 0, "kI must be non-negative");
    assertTrue(Constants.ShooterConstants.SHOOTER_kD >= 0, "kD must be non-negative");
    assertTrue(Constants.ShooterConstants.SHOOTER_kFF >= 0, "kFF must be non-negative");
  }

  @Test
  void shooterTargetRpmIsPositive() {
    assertTrue(Constants.ShooterConstants.SHOOTER_TARGET_RPM > 0);
    assertTrue(Constants.ShooterConstants.ACCELERATOR_TARGET_RPM > 0);
  }

  @Test
  void shooterToleranceIsPositive() {
    assertTrue(Constants.ShooterConstants.SHOOTER_TOLERANCE_RPM > 0);
  }

  @Test
  void currentLimitsAreWithinSafeRange() {
    assertTrue(Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT > 0);
    assertTrue(Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT <= 80);

    assertTrue(Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT > 0);
    assertTrue(Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT <= 80);

    assertTrue(Constants.FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT > 0);
    assertTrue(Constants.FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT > 0);
  }

  @Test
  void visionConstantsAreValid() {
    assertTrue(Constants.VisionConstants.AIM_TOLERANCE_DEGREES > 0);
    assertTrue(Constants.VisionConstants.AIM_KP > 0);
    assertTrue(Constants.VisionConstants.RANGE_KP > 0);
    assertTrue(Constants.VisionConstants.RANGE_TOLERANCE_METERS > 0);
    assertTrue(Constants.VisionConstants.MAX_POSE_AMBIGUITY > 0);
    assertTrue(Constants.VisionConstants.MAX_POSE_DISTANCE_METERS > 0);
  }

  @Test
  void reefscapeTagIdsAreNonEmpty() {
    assertTrue(Constants.VisionConstants.REEF_TAG_IDS_RED.length > 0);
    assertTrue(Constants.VisionConstants.REEF_TAG_IDS_BLUE.length > 0);
    assertTrue(Constants.VisionConstants.PROCESSOR_TAG_IDS.length > 0);
    assertTrue(Constants.VisionConstants.BARGE_TAG_IDS.length > 0);
  }

  @Test
  void scoringDistancesArePositive() {
    assertTrue(Constants.VisionConstants.REEF_SCORING_DISTANCE > 0);
    assertTrue(Constants.VisionConstants.PROCESSOR_SCORING_DISTANCE > 0);
    assertTrue(Constants.VisionConstants.BARGE_SCORING_DISTANCE > 0);
  }

  @Test
  void operatorDeadbandIsValid() {
    assertTrue(Constants.OperatorConstants.DEADBAND > 0);
    assertTrue(Constants.OperatorConstants.DEADBAND < 0.5);
  }

  @Test
  void driveScalingIsValid() {
    assertTrue(Constants.OperatorConstants.DRIVE_SCALING > 0);
    assertTrue(Constants.OperatorConstants.DRIVE_SCALING <= 1.0);
    assertTrue(Constants.OperatorConstants.DRIVE_TURBO_SCALING > 0);
    assertTrue(Constants.OperatorConstants.DRIVE_TURBO_SCALING <= 1.0);
    assertTrue(
        Constants.OperatorConstants.DRIVE_TURBO_SCALING
            >= Constants.OperatorConstants.DRIVE_SCALING);
  }
}
