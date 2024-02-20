// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class Intake
  {
    public static final double INTAKE_SPEED = 0.5;

    public static final double MIN_TILT = 0; //degrees

    public static final double MAX_TILT = 72.3; //degrees

    public static final double INTAKE_CONVERSION_FACTOR = 16/34*16;

    public static final PIDController tiltPIDcontrol = new PIDController(0.01, 0, 0);
  }

  public static class Shooter
  {
    //Maximum shooter angle
    public static final double MAX_TILT = 57.5; //degrees
    //Minimum shooter angle
    public static final double MIN_TILT = 10; //degrees
    //Maximum tilt speed
    public static final double MAX_TILT_SPEED = 0.2;

    public static final double TILT_CONVERSION_FACTOR = 50/16;

    public static final PIDController tiltPIDcontrol = new PIDController(0.02, 0, 0);

    public static final double STICK_DEADZONE = 0.1;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 6;
  }
}
