package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    // Climber Motors
    CANSparkMax leftClimber = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax rightClimber = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);
}
