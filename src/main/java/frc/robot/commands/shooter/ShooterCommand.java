package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommand extends Command {

  ShooterSubsystem shooterSubsystem;

  double speed;
  double diff;

  /**
   * Constructor for the ShooterCommand class.
   *
   * @param shooterSubsytem The ShooterSubsystem to use.
   */
  public ShooterCommand(
    ShooterSubsystem shooterSubsytem,
    double speed,
    double diff
    boolean ignoreBeamBrake,
    IndexerSubsystem rollers
  ) {
    // set the shooter to the ShooterSubsytem
    addRequirements(shooterSubsytem);
    // set the shooter to the ShooterSubsytem
    this.shooterSubsystem = shooterSubsytem;
    // set the speed to the speed
    this.speed = speed;
    // set the diff to the diff
    this.diff = diff;
    this.rollers = rollers;
    this.ignoreBeamBrake = ignoreBeamBrake;
  }

  /**
   * Method to execute the command.
   *
   * This method runs the shooter at a given speed.
   */
  @Override
  public void execute() {
    shooterSubsystem.runShooter(speed, false, diff);
  }

  /**
   * Method to end the command.
   *
   * This method stops the shooter.
   */
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
    // stop the shooter
    shooterSubsystem.runShooter(0, false, 0);
  }
}
