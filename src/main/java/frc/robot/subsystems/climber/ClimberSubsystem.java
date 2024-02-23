package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // Climber Motors
  CANSparkMax leftClimber = new CANSparkMax(
    15,
    CANSparkLowLevel.MotorType.kBrushless
  );
  CANSparkMax rightClimber = new CANSparkMax(
    16,
    CANSparkLowLevel.MotorType.kBrushless
  );

  RelativeEncoder leftEncoder = leftClimber.getEncoder();
  RelativeEncoder rightEncoder = rightClimber.getEncoder();

  public void runClimbers(double speed, boolean isBreakMode) {
    //leftClimber.set(-speed);
    rightClimber.set(speed);
    if (isBreakMode) {
      leftClimber.setIdleMode(IdleMode.kBrake);
      rightClimber.setIdleMode(IdleMode.kBrake);
    } else {
      leftClimber.setIdleMode(IdleMode.kCoast);
      rightClimber.setIdleMode(IdleMode.kCoast);
    }
    SmartDashboard.putNumber("Right Climber Encoder", getEncoders()[0]);
    SmartDashboard.putNumber("Left Encoder", getEncoders()[1]);
    leftClimber.setSmartCurrentLimit(20, 80);
    rightClimber.setSmartCurrentLimit(20, 80);
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public double[] getEncoders() {
    double[] result = { rightEncoder.getPosition(), leftEncoder.getPosition() };

    return result;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right Climber Encoder", getEncoders()[0]);
    SmartDashboard.putNumber("Left Climber Encoder", getEncoders()[1]);
    SmartDashboard.putNumber(
      "Right Climber Current",
      rightClimber.getOutputCurrent()
    );
    SmartDashboard.putNumber(
      "Left Climber Current",
      leftClimber.getOutputCurrent()
    );
  }
}
