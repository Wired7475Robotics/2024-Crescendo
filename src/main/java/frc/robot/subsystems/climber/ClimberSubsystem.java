package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    // Climber Motors
    CANSparkMax leftClimber = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax rightClimber = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);

    public void runClimbers(double speed, boolean isBreakMode){
        leftClimber.set(speed);
        rightClimber.set(speed);
        if (isBreakMode) {
            leftClimber.setIdleMode(IdleMode.kBrake);
            rightClimber.setIdleMode(IdleMode.kBrake);
        } else {
            leftClimber.setIdleMode(IdleMode.kCoast);
            rightClimber.setIdleMode(IdleMode.kCoast);
        }
    }
}
