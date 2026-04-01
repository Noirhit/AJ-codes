package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterSubsystemTest {

  private ShooterSubsystem shooter;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  void teardown() {
    CommandScheduler.getInstance().close();
    if (shooter != null) {
      shooter.close();
    }
  }

  @Test
  void subsystemInstantiatesWithoutError() {
    assertDoesNotThrow(() -> shooter = new ShooterSubsystem());
  }

  @Test
  void stopSetsMotorsToZero() {
    shooter = new ShooterSubsystem();
    shooter.setTargetRPM(4000);
    shooter.stop();
    assertEquals(0.0, shooter.getUpperRPM(), 0.01);
    assertEquals(0.0, shooter.getLowerRPM(), 0.01);
    assertEquals(0.0, shooter.getAcceleratorRPM(), 0.01);
  }

  @Test
  void isAtSpeedReturnsFalseWhenStopped() {
    shooter = new ShooterSubsystem();
    shooter.setTargetRPM(4000);
    assertFalse(shooter.isAtSpeed());
  }

  @Test
  void isAtSpeedReturnsTrueWhenTargetIsZero() {
    shooter = new ShooterSubsystem();
    shooter.stop();
    assertTrue(shooter.isAtSpeed());
  }
}
