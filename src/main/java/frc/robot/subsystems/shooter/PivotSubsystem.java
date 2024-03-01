package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.MathConstants;

public class PivotSubsystem extends SubsystemBase {

  // Tilt drive absolute encoder
  DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(2);
  // Motor for tilting the Shooter subsystem
  CANSparkMax tiltDrive = new CANSparkMax(10, MotorType.kBrushless);

  // Initialize the conversion factor for the tilt encoder
  double tiltConversionFactor;
  // Timer for filtering the tilt angle
  Timer tiltTimeout = new Timer();
  // booleans for state of filtering
  boolean timeout = true;
  boolean filtering = false;
  // last distance before timeout
  double lastDist;
  // Target value for the tilt angle
  double targetValue;

  /**
   * Constructor for the TiltSubsystem class.
   *
   * Initializes the tiltConversionFactor and sets the periodic method to run.
   */
  public PivotSubsystem() {
    // Set the conversion factor for the tilt encoder. done this way to save processing power later by avoiding redoing the math every time we need to get the conversion factor
    tiltConversionFactor = Constants.Shooter.TILT_CONVERSION_FACTOR;
    super.periodic();
  }

  /**
   * Method to run the tilt motor at a given speed.
   *
   * @param speed The speed to run the tilt motor at.
   */
  public void runTilt(double speed) {
    // Run the tilt motor at the given speed, as long as that speed is within the bounds of the max tilt speed
    tiltDrive.set(
      Math.min(
        Constants.Shooter.MAX_TILT_SPEED,
        Math.max(-Constants.Shooter.MAX_TILT_SPEED, speed)
      )
    );
  }

  /**
   * Method to reset the tilt encoder.
   */
  public void resetTiltEncoder() {
    // Reset the tilt encoder
    tiltEncoder.reset();
    // Set the target value to the current tilt angle
    targetValue = getTiltDegrees();
  }

  /**
   * Method to get the tilt angle in degrees.
   *
   * @return The tilt angle in degrees.
   */
  public double getTiltDegrees() {
    // Get the tilt angle in rotations
    double rotations = -tiltEncoder.get() / tiltConversionFactor;
    // Return the tilt angle in degrees. add 10 to account for the offset of the resting position of the shooter
    return (360 * rotations) + Shooter.MIN_TILT;
  }

  /**
   * Method to get the tilt angle in radians.
   *
   * @param aprilTagAngle The angle of the AprilTag (y axis).
   * @return The tilt angle in radians.
   */
  public double calculateTargetAngle(
    double aprilTagAngle,
    double robotVelocity
  ) {
    // Calculate the distance to the target
    double distance = 
        (MathConstants.APRIL_TAG_HEIGHT - MathConstants.CAMERA_HEIGHT) /
        Math.tan(Math.toRadians(aprilTagAngle + MathConstants.CAMERA_ANGLE));
    // Calculate the new target value
    double newTargetValue = Math.toDegrees(
      Math.atan(
        (MathConstants.TARGET_HEIGHT - MathConstants.SHOOTER_HEIGHT) /
        (
          MathConstants.TARGET_DISTANCE +
          MathConstants.SHOOTER_DISTANCE +
          distance -
          (distance * MathConstants.GRAVITY_CONSTANT) -
          (robotVelocity * MathConstants.VELOCITY_CONSTANT)
        )
      )
    );
    // If the difference between the new target value and the current target value is greater than 20, or the AprilTag angle is 0 (the limelight likely does not see an apriltag), and the timeout is not active, start filtering april tag angle to avoid sudden changes in the tilt angle
    if (
      (
        Math.abs(targetValue - newTargetValue) >
        Shooter.APRILTAG_MIN_FILTEROUT_ANGLE ||
        aprilTagAngle == 0
      ) &&
      !timeout
    ) {
      // If the filtering is not active, start the timer
      if (!filtering) {
        filtering = true;
        tiltTimeout.start();
      }
      // If the timer has been running for more than the set timeout in seconds, stop the filtering and set the timeout to true
      if (tiltTimeout.get() > Shooter.TIMEOUT) {
        filtering = false;
        timeout = true;
        tiltTimeout.stop();
        tiltTimeout.reset();
      }
      // return the old target value to avoid jittering movement
      return targetValue;
    }

    // Set the new target value to the calculated target value, as long as the new target value is within the bounds of the max and min tilt
    targetValue =
      Math.min(Shooter.MAX_TILT, Math.max(Shooter.MIN_TILT, newTargetValue));
    System.out.println(distance + "," + newTargetValue);
    // If the timeout is active and the april tag angle is 0 (the limelight likely does not see an apriltag), return the max tilt angle
    if (timeout && aprilTagAngle == 0) {
      return Shooter.MAX_TILT;
    }
    // If the timeout is active and the april tag angle is not 0, set the timeout to false
    timeout = false;
    // Set the last distance to the current distance
    lastDist = distance;
    // Return the new target value
    return targetValue;
  }

  /**
   * Method to calibrate the camera angle.
   *
   * Set an april tag 60 inches away from the camera and 16 inches off the ground, and this method will return the camera angle relative to the floor.
   *
   * @param aprilTagAngle The angle of the AprilTag (y axis).
   * @return The calibrated camera angle.
   */
  public double calibrateCameraAngle(double aprilTagAngle) {
    // Return the calibrated camera angle
    return (
      Math.toDegrees(
        Math.atan2(
          MathConstants.CALIBRATION_HEIGHT,
          MathConstants.CALIBRATION_DISTANCE
        )
      ) -
      aprilTagAngle
    );
  }

  /**
   * Method to check if the tilt subsystem is ready to shoot.
   *
   * @return Whether the tilt subsystem is ready to shoot.
   */
  public boolean isReady() {
    // If the timeout is active and the last distance is greater than the maximum distance, or the filtering is active, return false
    if ((timeout && lastDist > Shooter.MAX_DISTANCE) || filtering) {
      return false;
    }
    return true;
  }

  /**
   * For testing manually.
   * @param stickaxis double value of the joystick axis
   * @return double value of the target angle
   *
   */
  public double getTargetAngle(double stickaxis) {
    // If the target value is within the bounds of the max and min tilt, and the stick axis is greater than the deadzone, set the target value to the target value minus the stick axis
    if (
      Shooter.MIN_TILT < targetValue - stickaxis &&
      targetValue - stickaxis < Shooter.MAX_TILT &&
      (
        stickaxis > Shooter.STICK_DEADZONE ||
        stickaxis < -Shooter.STICK_DEADZONE
      )
    ) {
      targetValue = targetValue - stickaxis;
    }

    return targetValue;
  }

  /**
   * Periodic method for the TiltSubsystem class.
   *
   * Puts the tilt angle and target value on the SmartDashboard.
   */
  @Override
  public void periodic() {
    super.periodic();
    // Put the tilt angle and target value on the SmartDashboard for debugging
    SmartDashboard.putNumber("Shooter Tilt", getTiltDegrees());
    SmartDashboard.putNumber("Shooter Target", targetValue);
    SmartDashboard.putNumber(
      "ShooterTilt Error",
      getTiltDegrees() - targetValue
    );
  }
}
