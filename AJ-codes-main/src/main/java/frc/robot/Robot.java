package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.InputStream;
import java.util.Properties;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * Prints the project folder and build time that were stamped into this jar by
   * the generateBuildInfo task in build.gradle. Look for it in the Driver Station
   * console (or SmartDashboard) after each deploy to confirm which code is running.
   */
  private static void reportBuildInfo() {
    String project = "unknown";
    String built = "unknown";
    try (InputStream in = Robot.class.getResourceAsStream("/buildinfo.properties")) {
      if (in != null) {
        Properties props = new Properties();
        props.load(in);
        project = props.getProperty("project", project);
        built = props.getProperty("built", built);
      }
    } catch (Exception e) {
      // Never let a diagnostic stop the robot from starting.
    }
    System.out.println("==================== ROBOT CODE ====================");
    System.out.println("Project: " + project);
    System.out.println("Built:   " + built);
    System.out.println("====================================================");
    SmartDashboard.putString("Code Project", project);
    SmartDashboard.putString("Code Built", built);
  }

  @Override
  public void robotInit() {
    // Show which project folder and build is actually running on the robot.
    reportBuildInfo();

    m_robotContainer = new RobotContainer();

    HAL.report(tResourceType.kResourceType_Framework, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
     CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
