package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase{

    // Motor for tilting the Shooter subsystem
    CANSparkMax tiltDrive = new CANSparkMax(10, MotorType.kBrushless);

    // Motor driving the roller-feed
    CANSparkMax noteFeed = new CANSparkMax(12, MotorType.kBrushless);
    // Shooters
    CANSparkMax shooterLeft = new CANSparkMax(13, MotorType.kBrushless);
    CANSparkMax shooterRight = new CANSparkMax(14, MotorType.kBrushless);

    public Command runShooter = new Command() {
        public void execute() {
            shooterLeft.set(0.9);
            shooterRight.set(-0.9);
        };
        public void end(boolean interrupted) {
            shooterLeft.set(0);
            shooterRight.set(0);
        };
    };

}
