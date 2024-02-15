package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {

  // Motor for tilting the Shooter subsystem
  CANSparkMax tiltDrive = new CANSparkMax(10, MotorType.kBrushless);

  // Motor driving the roller-feed
  CANSparkMax noteFeed = new CANSparkMax(12, MotorType.kBrushless);
  // Shooters
  CANSparkMax shooterLeft = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax shooterRight = new CANSparkMax(14, MotorType.kBrushless);

  DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(2);

  public void runShooter(double speed, boolean isBreakMode) {
    shooterLeft.set(-speed);
    shooterRight.set(speed);
    if (isBreakMode) {
      shooterLeft.setIdleMode(IdleMode.kBrake);
      shooterRight.setIdleMode(IdleMode.kBrake);
    } else {
      shooterLeft.setIdleMode(IdleMode.kCoast);
      shooterRight.setIdleMode(IdleMode.kCoast);
    }
  }

  public void runTilt(double speed) {
    if (
      Shooter.MIN_TILT < getTiltDegrees() && getTiltDegrees() < Shooter.MAX_TILT
    ) {
      tiltDrive.set(speed);
    } else {
      System.err.print("Shooter tried to go oudside of its bounds");
    }
  }

  public void runRollers(boolean ignoreBeamBrake, double speed) {
    noteFeed.set(speed);
  }

  public void resetTiltEncoder() {
    double offset = tiltEncoder.get();
    tiltEncoder.setPositionOffset(offset);
  }

  public double getTiltDegrees() {
    double rotations = tiltEncoder.get();
    return 360 / rotations - 10;
  }

  public double getTargetAngle() {
    return 0;
  }

  /**
   * For testing manually.
   * @param stickaxis
   * @return
   *
   */
  public double getTargetAngle(double stickaxis) {
    return getTiltDegrees() + stickaxis;
  }
}
