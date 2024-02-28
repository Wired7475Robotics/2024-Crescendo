package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  // Motor driving the roller-feed

  // Shooters
  CANSparkMax shooterLeft = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax shooterRight = new CANSparkMax(14, MotorType.kBrushless);

  public ShooterSubsystem() {
    shooterLeft.setOpenLoopRampRate(2);
    shooterRight.setOpenLoopRampRate(2);
  }

  public void runShooter(double speed, boolean isBreakMode, double diff) {
    shooterLeft.set(speed);
    shooterRight.set(-speed+diff);
    if (isBreakMode) {
      shooterLeft.setIdleMode(IdleMode.kBrake);
      shooterRight.setIdleMode(IdleMode.kBrake);
    } else {
      shooterLeft.setIdleMode(IdleMode.kCoast);
      shooterRight.setIdleMode(IdleMode.kCoast);
    }
  }
}
