package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class DeployerSubsystem extends SubsystemBase {

  double targetValue;

  double tiltConversionFactor;
  //Motor for deploying intake
  CANSparkMax tiltDrive = new CANSparkMax(
    9,
    CANSparkLowLevel.MotorType.kBrushless
  );

  RelativeEncoder tiltEncoder = tiltDrive.getEncoder();

  public DeployerSubsystem() {
    tiltConversionFactor = Constants.Intake.INTAKE_CONVERSION_FACTOR;
    tiltDrive.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("Intake Tilt", getTiltDegrees());
    SmartDashboard.putNumber("Intake Target", targetValue);
    SmartDashboard.putNumber(
      "IntakeTilt Error",
      getTiltDegrees() - targetValue
    );
  }

  public void runTilt(double speed) {
    tiltDrive.set(
      Math.min(
        Constants.Intake.INTAKE_SPEED,
        Math.max(-Constants.Intake.INTAKE_SPEED, speed)
      )
    );
  }

  public void resetTiltEncoder() {
    tiltEncoder.setPosition(0);
    targetValue = 0;
  }

  public double getTiltDegrees() {
    double rotations = tiltEncoder.getPosition() / tiltConversionFactor;
    return (360 * rotations);
  }

  /**
   * For testing manually.
   * @param stickaxis
   * @return
   *
   */
  public double getTargetAngle(double stickaxis) {
    if (
      Intake.MIN_TILT < targetValue + stickaxis &&
      targetValue + stickaxis < Intake.MAX_TILT &&
      (stickaxis > Intake.STICK_DEADZONE || stickaxis < -Intake.STICK_DEADZONE)
    ) {
      targetValue = targetValue + stickaxis;
    } else {
      System.out.println("Intake tried to go oudside of its bounds");
    }
    return targetValue;
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Intake Tilt", getTiltDegrees());
    SmartDashboard.putNumber("Intake Target", targetValue);
    SmartDashboard.putNumber(
      "IntakeTilt Error",
      getTiltDegrees() - targetValue
    );
  }
}
