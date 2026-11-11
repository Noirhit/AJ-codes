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
      Constants.DriveConstants.RIGHT_FOLLOWER_ID
    };

    for (int id : allIds) {
      assertTrue(ids.add(id), "Duplicate CAN ID found: " + id);
    }

    assertEquals(6, ids.size());
  }

  @Test
  void canIdsAreInValidRange() {
    int[] allIds = {
      Constants.FuelConstants.FEEDER_MOTOR_ID,
      Constants.FuelConstants.INTAKE_LAUNCHER_MOTOR_ID,
      Constants.DriveConstants.LEFT_LEADER_ID,
      Constants.DriveConstants.LEFT_FOLLOWER_ID,
      Constants.DriveConstants.RIGHT_LEADER_ID,
      Constants.DriveConstants.RIGHT_FOLLOWER_ID
    };

    for (int id : allIds) {
      assertTrue(id >= 1 && id <= 62, "CAN ID out of valid range (1-62): " + id);
    }
  }

  @Test
  void currentLimitsAreWithinSafeRange() {
    assertTrue(Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT > 0);
    assertTrue(Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT <= 80);

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
  void hubTagIdsAreNonEmpty() {
    assertTrue(Constants.VisionConstants.HUB_TAG_IDS.length > 0);
  }

  @Test
  void scoringDistancesArePositive() {
    assertTrue(Constants.VisionConstants.HUB_SCORING_DISTANCE > 0);
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
    assertTrue(Constants.OperatorConstants.DRIVE_SLOW_SCALING > 0);
    assertTrue(Constants.OperatorConstants.DRIVE_SLOW_SCALING <= 1.0);
    assertTrue(
        Constants.OperatorConstants.DRIVE_SLOW_SCALING
            <= Constants.OperatorConstants.DRIVE_SCALING);
  }
}
