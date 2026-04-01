package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LaunchSequenceTest {

  private ShooterSubsystem shooter;
  private CANFuelSubsystem fuel;

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
    if (fuel != null) {
      fuel.close();
    }
  }

  @Test
  void launchSequenceInstantiates() {
    fuel = new CANFuelSubsystem();
    assertDoesNotThrow(() -> new LaunchSequence(fuel));
  }

  @Test
  void shooterLaunchSequenceInstantiates() {
    shooter = new ShooterSubsystem();
    fuel = new CANFuelSubsystem();
    assertDoesNotThrow(() -> new ShooterLaunchSequence(shooter, fuel));
  }
}
