package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

  public DeployerSubsystem() {
    tiltConversionFactor = Constants.Intake.INTAKE_CONVERSION_FACTOR;
    tiltDrive.setIdleMode(IdleMode.kBrake);

    tiltDrive.setSmartCurrentLimit(50);
  }

  public void runTilt(double speed) {
    tiltDrive.set(
      Math.min(
        Constants.Intake.INTAKE_SPEED,
        Math.max(-Constants.Intake.INTAKE_SPEED, speed)
      )
    );

    System.out.println(speed);
  }

  public void runTiltSlow() {
    runTilt(-0.025);
  }

  public double getVelocity() {
    return tiltDrive.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("TiltCurrent", tiltDrive.getOutputCurrent());
    SmartDashboard.putNumber("IntakeVelocity", getVelocity());
  }
}
