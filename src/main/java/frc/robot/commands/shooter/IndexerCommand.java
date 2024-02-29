package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.IndexerSubsystem;

public class IndexerCommand extends Command {

  // Initialize the IndexerSubsystem, ignoreBeamBrake, and speed
  IndexerSubsystem rollers;
  boolean ignoreBeamBrake;
  double speed;

  /**
   * Constructor for the IndexerCommand class.
   *
   * @param rollers The IndexerSubsystem to use.
   * @param ignoreBeamBrake Whether or not to ignore the beam break sensor.
   * @param speed The speed to run the indexer motor at.
   */
  public IndexerCommand(
    IndexerSubsystem rollers,
    boolean ignoreBeamBrake,
    double speed
  ) {
    // Add the IndexerSubsystem to the requirements of the command, and set the rollers, ignoreBeamBrake, and speed
    addRequirements(rollers);
    this.rollers = rollers;
    this.ignoreBeamBrake = ignoreBeamBrake;
    this.speed = -speed;
  }

  @Override
  public void execute() {
    // Run the indexer motor at the given speed
    rollers.runRollers(speed);
  }

  @Override
  public boolean isFinished() {
    // Return true if the beam break sensor is broken and ignoreBeamBrake is false
    if (!ignoreBeamBrake && !rollers.getBeamBreak()) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the indexer motor
    rollers.runRollers(0);
  }
}
