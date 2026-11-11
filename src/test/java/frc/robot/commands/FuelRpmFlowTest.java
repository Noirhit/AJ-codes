package frc.robot.commands;

import static frc.robot.Constants.FuelConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CANFuelSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Robot-free check that the RPM you type into SmartDashboard is what actually gets commanded to
 * the motor controllers. REVLib exposes each SPARK MAX in simulation with a "Setpoint" field (the
 * RPM being commanded), which is the same value you see in the Sim GUI's "Other Devices" window.
 */
class FuelRpmFlowTest {
  private static final double EPS = 1e-9;

  private CANFuelSubsystem fuel;
  private SimDeviceSim launcherMotor;
  private SimDeviceSim feederMotor;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    fuel = new CANFuelSubsystem(); // also publishes the default SmartDashboard values
    launcherMotor = new SimDeviceSim("SPARK MAX [" + INTAKE_LAUNCHER_MOTOR_ID + "]");
    feederMotor = new SimDeviceSim("SPARK MAX [" + FEEDER_MOTOR_ID + "]");
  }

  @AfterEach
  void teardown() {
    CommandScheduler.getInstance().close();
    fuel.close();
  }

  private double launcherSetpoint() {
    return launcherMotor.getDouble("Setpoint").get();
  }

  private double feederSetpoint() {
    return feederMotor.getDouble("Setpoint").get();
  }

  @Test
  void teleopLaunchFollowsSmartDashboard() {
    Launch launch = new Launch(fuel);
    launch.initialize();

    // Untouched dashboard -> the Constants defaults.
    launch.execute();
    assertEquals(LAUNCHING_LAUNCHER_RPM, launcherSetpoint(), EPS);
    assertEquals(LAUNCHING_FEEDER_RPM, feederSetpoint(), EPS);

    // Slow the launcher down (closer to zero) -> the commanded RPM drops with it.
    SmartDashboard.putNumber(CANFuelSubsystem.KEY_LAUNCHING_LAUNCHER, -3000);
    launch.execute();
    assertEquals(-3000, launcherSetpoint(), EPS);

    SmartDashboard.putNumber(CANFuelSubsystem.KEY_LAUNCHING_LAUNCHER, -2000);
    launch.execute();
    assertEquals(-2000, launcherSetpoint(), EPS);

    // The feeder has its own value.
    SmartDashboard.putNumber(CANFuelSubsystem.KEY_LAUNCHING_FEEDER, -1500);
    launch.execute();
    assertEquals(-1500, feederSetpoint(), EPS);
  }

  @Test
  void spinUpUsesLauncherValueAndSpinUpFeederValue() {
    SpinUp spinUp = new SpinUp(fuel);
    spinUp.initialize();

    spinUp.execute();
    assertEquals(LAUNCHING_LAUNCHER_RPM, launcherSetpoint(), EPS);
    assertEquals(SPIN_UP_FEEDER_RPM, feederSetpoint(), EPS);

    SmartDashboard.putNumber(CANFuelSubsystem.KEY_LAUNCHING_LAUNCHER, -3500);
    SmartDashboard.putNumber(CANFuelSubsystem.KEY_SPINUP_FEEDER, 300);
    spinUp.execute();
    assertEquals(-3500, launcherSetpoint(), EPS);
    assertEquals(300, feederSetpoint(), EPS);
  }

  @Test
  void intakeFollowsSmartDashboard() {
    Intake intake = new Intake(fuel);
    intake.initialize();

    intake.execute();
    assertEquals(INTAKING_INTAKE_RPM, launcherSetpoint(), EPS);
    assertEquals(INTAKING_FEEDER_RPM, feederSetpoint(), EPS);

    SmartDashboard.putNumber(CANFuelSubsystem.KEY_INTAKING_INTAKE, -1500);
    SmartDashboard.putNumber(CANFuelSubsystem.KEY_INTAKING_FEEDER, 1200);
    intake.execute();
    assertEquals(-1500, launcherSetpoint(), EPS);
    assertEquals(1200, feederSetpoint(), EPS);
  }

  @Test
  void autoLauncherUsesFixedRpmAndIgnoresSmartDashboardLauncherValue() {
    // Auto passes a fixed RPM (AUTO_LAUNCHING_LAUNCHER_RPM from Constants), so editing the
    // teleop "Launching launcher roller value" on the dashboard must NOT change it.
    SmartDashboard.putNumber(CANFuelSubsystem.KEY_LAUNCHING_LAUNCHER, -1000);
    SpinUp autoSpinUp = new SpinUp(fuel, () -> AUTO_LAUNCHING_LAUNCHER_RPM);
    autoSpinUp.initialize();
    autoSpinUp.execute();
    assertEquals(AUTO_LAUNCHING_LAUNCHER_RPM, launcherSetpoint(), EPS);
  }
}
