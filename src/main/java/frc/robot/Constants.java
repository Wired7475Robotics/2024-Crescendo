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
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound

  public static final Matter CHASSIS = new Matter(
    new Translation3d(0, 0, Units.inchesToMeters(8)),
    ROBOT_MASS
  );

  public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);

    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final double FAST_SPEED = 1;

    public static final double MEDIUM_SPEED = 0.75;

    public static final double SLOW_SPEED = 0.6;
  }

  public static class Intake {

    public static final double INTAKE_SPEED = 1;

    public static final double MIN_TILT = 0; //degrees

    public static final double MAX_TILT = 75.3; //degrees

    public static final double INTAKE_CONVERSION_FACTOR = 53.125; // 25/16*34

    public static final PIDController tiltPIDcontrol = new PIDController(
      0.06,
      0,
      0.000
    );

    public static final int HIGH = 1;

    public static final int LOW = 0;

   
  }

  public static class Shooter {

    //Maximum shooter angle
    public static final double MAX_TILT = 62; //degrees
    //Minimum shooter angle
    public static final double MIN_TILT = 10; //degrees
    //Maximum tilt speed
    public static final double MAX_TILT_SPEED = 1;

    public static final double TILT_CONVERSION_FACTOR = 50 / 16;

    public static final PIDController tiltPIDcontrol = new PIDController(
      0.015,
      0,
      0.0002
    );

    public static class MathConstants {

      public static final double APRIL_TAG_HEIGHT = 57.125;

      public static final double TARGET_HEIGHT = 82.875;

      public static final double TARGET_DISTANCE = 2;

      public static final double SHOOTER_DISTANCE = 5.3428;

      public static final double SHOOTER_HEIGHT = 6.906585;

      public static final double CAMERA_HEIGHT = 4.429365;

      public static final double CAMERA_ANGLE = 19.03;

      public static final double GRAVITY_CONSTANT = 0.01;

      public static final double CALIBRATION_DISTANCE = 60;

      public static final double CALIBRATION_HEIGHT = 16;
    }

    public static final double STICK_DEADZONE = 0.1;
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;

    public static final double LEFT_Y_DEADBAND = 0.02;

    public static final double RIGHT_X_DEADBAND = 0.01;

    public static final double TURN_CONSTANT = 6;

    public static final int FALSE = 0;
    
    public static final int NULL = 1;

    public static final int TRUE = 2; 

    
  }
}
