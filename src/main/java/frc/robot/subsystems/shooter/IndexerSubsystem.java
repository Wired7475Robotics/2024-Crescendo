package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  // initialize the indexer motor
  CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushless);

  // initialize the beam break sensor
  DigitalInput beamBreak = new DigitalInput(1);

  /**
   * Constructor for the IndexerSubsystem class.
   *
   * Initializes the indexer motor and sets the idle mode for the motor.
   */
  public IndexerSubsystem() {
    // Set the idle mode for the indexer motor to brake
    indexer.setIdleMode(IdleMode.kBrake);
    // set the periodic frame period for the indexer motor to 0 for several status reports to avoid CAN bus overload
    indexer.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    indexer.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    indexer.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0);
    indexer.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    indexer.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);
  }

  /**
   * Method to run the indexer motor at a given speed.
   *
   * @param speed The speed to run the indexer motor at.
   */
  public void runRollers(double speed) {
    // Run the indexer motor at the given speed
    indexer.set(speed);
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
