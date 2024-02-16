package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
    CANSparkMax noteFeed = new CANSparkMax(12, MotorType.kBrushless);

    public RollerSubsystem(){
        
    }

    public void runRollers(boolean ignoreBeamBrake, double speed) {
        noteFeed.set(speed);
      }
}
