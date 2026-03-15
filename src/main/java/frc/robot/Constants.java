// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 5;
    public static final int RIGHT_FOLLOWER_ID = 6;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 1;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 2;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12; //intake rear, left trigger, counterclockwise (metal side facing left)
    public static final double INTAKING_INTAKE_VOLTAGE = 10; //intake front, left trigger, counterclockwise (metal side facing left)
    // if the orange wheel is not upto speed decrease this value so that it gets enough time to speed up to throw farther
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9; //launch rear, right trigger, counterclockwise (metal side facing left) - negative pushes ball toward launcher
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 12; //launch front, right trigger, counterclockwise (metal side facing left)
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    // try to increase the speed so that the orange wheel gets time to fully get upto speed
    public static final double SPIN_UP_SECONDS = 1.5;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = .6;
    public static final double ROTATION_SCALING = .5;
    public static final double DRIVE_TURBO_SCALING = .9;
    public static final double ROTATION_TURBO_SCALING = .8;

    public static final double DEADBAND = 0.05;  // Exponent for rotation input squaring
    public static final double ROTATION_SENSITIVITY = 2.0;// Deadband for joystick inputs. This helps prevent the robot from moving when the joystick is near the center position.
    public static final double DRIVE_SENSITIVITY = 2.0; // Exponent for drive input squaring. Higher values give finer control at low speeds.
  }
}