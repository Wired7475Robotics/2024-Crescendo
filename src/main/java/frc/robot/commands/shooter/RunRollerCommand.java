package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.RollerSubsystem;

public class RunRollerCommand extends Command {

  RollerSubsystem rollers;
  boolean ignoreBeamBrake;
  double speed;

  public RunRollerCommand(
    RollerSubsystem rollers,
    boolean ignoreBeamBrake,
    double speed
  ) {
    addRequirements(rollers);
    this.rollers = rollers;
    this.ignoreBeamBrake = ignoreBeamBrake;
    this.speed = speed;
  }

  @Override
  public void execute() {
    rollers.runRollers(speed);
  }

  @Override
  public boolean isFinished() {
    if (!ignoreBeamBrake && rollers.getBeamBreak()) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    rollers.runRollers(0);
  }
}
