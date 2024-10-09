// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public static final double ROBOT_MASS = (124.4) * 0.453592; // 32lbs * kg per pound

  public static final Matter CHASSIS = new Matter(
    new Translation3d(0, 0, Units.inchesToMeters(8)),
    ROBOT_MASS
  );

  public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {

    public static final PIDFConfig translationPID = new PIDFConfig(0.7, 0, 0);

    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.0);

    public static final double MAX_ACCELERATION = 3;
  }

  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(
      0.7,
      0,
      0
    );
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    // Drivebase motor speed constants
    public static final double FAST_SPEED = 1; // percentage

    public static final double MEDIUM_SPEED = 0.75; // percentage

    public static final double SLOW_SPEED = 0.6; // percentage

    // Constant for autoaim to add to target angle to account for robot velocity
    public static final double VELOCITY_CONSTANT = 0.15; // arbitrary unit

    public static final double AIMING_TIME = 0.2;
  }

  public static class Intake {

    // Intake motor speed constants
    public static final double INTAKE_SPEED = 1; // percentage

    // Intake deploy conversion factor
    public static final double INTAKE_CONVERSION_FACTOR = 53.125; // 25/16*34, degrees per rotation

    // Intake status constants
    public static final int HIGH = 1;

    public static final int LOW = 0;

    public static final double SLOW_SPEED = 0.25; // percentage

    public static final double RAISE_SPEED = -0.5; // percentage

    public static final double LOWER_SPEED = 0.3; // percentage

    public static final double MIN_SPEED = 100; // RPM
  }

  public static class Shooter {

    //Maximum shooter angle
    public static final double MAX_TILT = 57.5; //degrees
    //Minimum shooter angle
    public static final double MIN_TILT = 10; //degrees
    //Maximum tilt speed
    public static final double MAX_TILT_SPEED = 0.4; //percentage
    //Minimum tilt RPM (when do we say that the motor has stalled)
    public static final double MIN_TILT_RPM = 75;
    // Tilt encoder offset
    public static final double OFFSET = 0.537791513444788; //degrees
    // Tilt conversion factor
    public static final double TILT_CONVERSION_FACTOR = 1; // degrees per rotation

    public static final double[] MathConstants = {
      0.04376,
      1.1457,
      -15.9901,
      70.83105
    };

    public static final double ShooterTable[][] = {
      { 0.95, 57.5 },
      { 1.3, 57.5 },
      { 1.6, 50 },
      { 1.7, 50 },
      { 2, 46 },
      { 2.3, 41.8 },
      { 2.5, 42.9 },
      { 2.7, 38 },
      { 2.8, 38.5 },
      { 2.9, 38.8 },
      { 3.25, 33.5 },
      { 3.5, 34.9 },
      { 3.75, 30.95 },
      { 3.9, 29.35 },
      { 4.1, 31.395 },
      { 4.15, 29.03 },
      { 4.5, 30.7 },
      { 4.6, 28.11 },
      { 4.9, 27.77 },
      { 5.1, 22.56 },
      { 6, 28.48 },
    };

    public static double getAngleFromTable(double dist) {
      int index = 0;
      double target = 0;
      for (int idx = 0; idx < ShooterTable.length; idx++) {
        if (ShooterTable[idx][0] <= dist) {
          index = idx;
          break;
        }
      }
      if (index == 0) target = ShooterTable[0][0]; else {
        double slope =
          (ShooterTable[index][1] - ShooterTable[index - 1][1]) /
          (ShooterTable[index][0] - ShooterTable[index - 1][0]);
        target =
          slope *
          dist +
          ((ShooterTable[index][1]) - slope * ShooterTable[index][0]);
      }
      SmartDashboard.putNumber("Shooter Table Value", index);
      return target;
    }

    // Shooter PID constants
    public static final PIDController pivotPidController = new PIDController(
      0.019,
      0.075,
      0.0009
    );

    // Variables for shooter autoaim logic
    public static final double TIMEOUT = 2; // seconds

    public static final double APRILTAG_MIN_FILTEROUT_ANGLE = 20; // degrees

    public static final double MAX_DISTANCE = 225.00; // inches

    public static final double MIN_SHOOTING_RPM = 4500; // rpm

    public static class MathConstants {

      // Constants for shooter autoaim math
      public static final double GRAVITY_CONSTANT = 0.0195; // arbitrary unit

      public static final double CALIBRATION_DISTANCE = 60; // inches

      public static final double CALIBRATION_HEIGHT = 16; // inches
    }

    //shooter stick deadzone for manual control
    public static final double STICK_DEADZONE = 0.1; // percentage
    public static final double AIMING_TIME = 0.05;
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;

    public static final double LEFT_Y_DEADBAND = 0.02;

    public static final double RIGHT_X_DEADBAND = 0.01;

    public static final double TURN_CONSTANT = 6;

    // note storage values
    public static final int FALSE = 0;

    public static final int NULL = 1;

    public static final int TRUE = 2;
  }
}
