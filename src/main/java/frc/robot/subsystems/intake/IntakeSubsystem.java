package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel;

public class IntakeSubsystem extends SubsystemBase{
    //Motor for deploying intake
    CANSparkMax tiltDrive = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless);
    
    //Motor for running the intake rollers
    CANSparkMax intake = new CANSparkMax(11,CANSparkLowLevel.MotorType.kBrushless);

    public void runIntakeRollers(double speed){
        intake.set(speed);
    }

}
