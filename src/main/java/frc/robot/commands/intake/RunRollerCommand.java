package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.RollerSubsystem;

public class RunRollerCommand extends Command {

  IntakeSubsystem rollers;
  boolean ignoreBeamBrake;
  double speed;

  boolean triggeredOnce = false;

  public RunRollerCommand(
    IntakeSubsystem rollers,
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
    rollers.runIntakeRollers(speed);
  }

  @Override
  public boolean isFinished() {
    if (!ignoreBeamBrake) {
      if (triggeredOnce && rollers.getBeamBreak()) {
        return true;
      }
      if (!rollers.getBeamBreak()) {
        triggeredOnce = true;
      }
      
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    rollers.runIntakeRollers(0);
    triggeredOnce = false;
  }
}
