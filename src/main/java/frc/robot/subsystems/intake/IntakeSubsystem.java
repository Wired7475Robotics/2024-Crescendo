package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  //Motor for running the intake rollers
  CANSparkMax intake = new CANSparkMax(
    11,
    CANSparkLowLevel.MotorType.kBrushless
  );

  DigitalInput beamBreak = new DigitalInput(0);

  public IntakeSubsystem() {
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);
  }

  public void runIntakeRollers(double speed) {
    intake.set(speed);
  }

public boolean getBeamBreak() {
    return beamBreak.get();
}
}
