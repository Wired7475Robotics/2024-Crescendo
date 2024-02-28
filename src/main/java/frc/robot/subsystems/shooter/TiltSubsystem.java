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

public class TiltSubsystem extends SubsystemBase {

  // Tilt drive absolute encoder
  DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(2);
  // Motor for tilting the Shooter subsystem
  CANSparkMax tiltDrive = new CANSparkMax(10, MotorType.kBrushless);

  double tiltConversionFactor;

  Timer tiltTimeout = new Timer();
  boolean timeout = true;
  boolean filtering = false;
  double lastDist;
  double targetValue;

  public TiltSubsystem() {
    tiltConversionFactor = Constants.Shooter.TILT_CONVERSION_FACTOR;

    super.periodic();
    SmartDashboard.putNumber("Shooter Tilt", getTiltDegrees());
    SmartDashboard.putNumber("Shooter Target", targetValue);
    SmartDashboard.putNumber(
      "ShooterTilt Error",
      getTiltDegrees() - targetValue
    );
  }

  public void runTilt(double speed) {
    tiltDrive.set(
      Math.min(
        Constants.Shooter.MAX_TILT_SPEED,
        Math.max(-Constants.Shooter.MAX_TILT_SPEED, speed)
      )
    );
  }

  public void resetTiltEncoder() {
    tiltEncoder.reset();
    targetValue = getTiltDegrees();
  }

  public double getTiltDegrees() {
    double rotations = -tiltEncoder.get() / tiltConversionFactor;
    return (360 * rotations) + 10;
  }

  public double calculateTargetAngle(double aprilTagAngle) {
    double distance = Math.sqrt(
      Math.pow(
        (MathConstants.APRIL_TAG_HEIGHT - MathConstants.CAMERA_HEIGHT) /
        Math.tan(Math.toRadians(aprilTagAngle + MathConstants.CAMERA_ANGLE)),
        2
      )
    );
    double newTargetValue = Math.toDegrees(
      Math.atan(
        (MathConstants.TARGET_HEIGHT - MathConstants.SHOOTER_HEIGHT) /
        (
          MathConstants.TARGET_DISTANCE +
          MathConstants.SHOOTER_DISTANCE +
          distance -
          (distance * MathConstants.GRAVITY_CONSTANT)
        )
      )
    );

    if (
      (Math.abs(targetValue - newTargetValue) > 20 || aprilTagAngle == 0) &&
      !timeout
    ) {
      if (!filtering) {
        filtering = true;
        tiltTimeout.start();
      }
      if (tiltTimeout.get() > 0.5) {
        filtering = false;
        timeout = true;
        tiltTimeout.stop();
        tiltTimeout.reset();
      }
      return targetValue;
    }

    targetValue =
      Math.min(Shooter.MAX_TILT, Math.max(Shooter.MIN_TILT, newTargetValue));
    System.out.println(distance + "," + newTargetValue);
    if (timeout && aprilTagAngle == 0) {
      return Shooter.MAX_TILT;
    }

    timeout = false;

    lastDist = distance;

    return targetValue;
  }

  public double calibrateCameraAngle(double aprilTagAngle) {
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

  public boolean isReady() {
    if ((timeout && lastDist < 150) || filtering) {
      return false;
    }
    return true;
  }

  /**
   * For testing manually.
   * @param stickaxis
   * @return
   *
   */
  public double getTargetAngle(double stickaxis) {
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

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Shooter Tilt", getTiltDegrees());
    SmartDashboard.putNumber("Shooter Target", targetValue);
    SmartDashboard.putNumber(
      "ShooterTilt Error",
      getTiltDegrees() - targetValue
    );
  }
}
