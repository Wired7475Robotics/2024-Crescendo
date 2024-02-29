package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command {

  // Initialize the intake subsystem and the variables for the command
  IntakeSubsystem rollers;
  boolean ignoreBeamBrake;
  double speed;

  // Counter for the beam break sensor
  boolean triggeredOnce = false;

  /**
   * Constructor for the IntakeCommand class.
   *
   * @param rollers The intake subsystem to be used by this command.
   * @param ignoreBeamBrake Whether or not to ignore the beam break sensor.
   * @param speed The speed to run the intake rollers at.
   */
  public IntakeCommand(
    IntakeSubsystem rollers,
    boolean ignoreBeamBrake,
    double speed
  ) {
    // Add the intake subsystem to the command's requirements and set the variables
    addRequirements(rollers);
    this.rollers = rollers;
    this.ignoreBeamBrake = ignoreBeamBrake;
    this.speed = speed;
  }

  /**
   * Method to run the intake rollers at the given speed.
   *
   * @param speed The speed to run the intake rollers at.
   */
  @Override
  public void execute() {
    // Run the intake rollers at the given speed
    rollers.runIntakeRollers(speed);
  }

  /**
   * Method to check if the command is finished.
   *
   * @return Whether or not the command is finished.
   */
  @Override
  public boolean isFinished() {
    // If the beam break sensor is triggered once and then released, and the command is not ignoring the beam break sensor, return true
    if (!ignoreBeamBrake) {
      if (triggeredOnce && rollers.getBeamBreak()) {
        return true;
      }
      if (!rollers.getBeamBreak()) {
        triggeredOnce = true;
      }
    }
    // Otherwise, return false
    return false;
  }

  /**
   * Method to end the command.
   *
   * @param interrupted Whether or not the command was interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the intake rollers and reset the beam break sensor counter
    rollers.runIntakeRollers(0);
    triggeredOnce = false;
  }
}
