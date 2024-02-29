package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // initialize the intake motor
  CANSparkMax intake = new CANSparkMax(
    11,
    CANSparkLowLevel.MotorType.kBrushless
  );
  // initialize the beam break sensor
  DigitalInput beamBreak = new DigitalInput(0);

  /**
   * Constructor for the IntakeSubsystem class.
   *
   * Initializes the intake motor and sets the periodic frame period for the motor to 0 for several status reports to avoid CAN bus overload.
   */
  public IntakeSubsystem() {
    // set the periodic frame period for the intake motor to 0 for several status reports to avoid CAN bus overload
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);
  }

  /**
   * Method to run the intake motor at a given speed.
   *
   * @param speed The speed to run the intake motor at.
   */
  public void runIntakeRollers(double speed) {
    // Run the intake motor at the given speed
    intake.set(speed);
  }

  /**
   * Method to check if the beam break sensor is broken.
   *
   * @return Whether or not the beam break sensor is not broken.
   */
  public boolean getBeamBreak() {
    // Return true if the beam break sensor is not broken
    return beamBreak.get();
  }
}
