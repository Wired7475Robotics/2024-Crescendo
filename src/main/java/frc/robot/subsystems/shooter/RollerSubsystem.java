package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {

  CANSparkMax noteFeed = new CANSparkMax(12, MotorType.kBrushless);

  DigitalInput beamBreak = new DigitalInput(1);

  public RollerSubsystem() {
    noteFeed.setIdleMode(IdleMode.kBrake);
    noteFeed.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    noteFeed.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    noteFeed.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0);
    noteFeed.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    noteFeed.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);
  }

  public void runRollers(double speed) {
    noteFeed.set(speed);
  }

  public boolean getBeamBreak() {
    return beamBreak.get();
  }
}
