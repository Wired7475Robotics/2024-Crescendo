package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.MathConstants;
import java.math.MathContext;
import org.opencv.core.Mat;

public class TiltSubsystem extends SubsystemBase {

  // Tilt drive absolute encoder
  DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(2);
  // Motor for tilting the Shooter subsystem
  CANSparkMax tiltDrive = new CANSparkMax(10, MotorType.kBrushless);

  double targetValue;

  double tiltConversionFactor;

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

  public double calculateTargetAngle(double AprilTagAngle) {
    double distance =
      Math.tan(AprilTagAngle + MathConstants.CAMERA_ANGLE) /
      (MathConstants.APRIL_TAG_HEIGHT - MathConstants.CAMERA_HEIGHT);

    return Math.atan(
      (MathConstants.TARGET_HEIGHT - MathConstants.SHOOTER_HEIGHT) /
      (MathConstants.TARGET_DISTANCE + MathConstants.SHOOTER_DISTANCE) +
      distance -
      (distance * MathConstants.GRAVITY_CONSTANT)
    );
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
    } else {
      System.out.println("Shooter tried to go oudside of its bounds");
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
