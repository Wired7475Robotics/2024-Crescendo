package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.shooter.PivotSubsystem;

public class PivotResetCommand extends Command {

  PivotSubsystem pivotSubsystem;
  Boolean moving = false;

  public PivotResetCommand(PivotSubsystem pivot) {
    pivotSubsystem = pivot;
  }

  @Override
  public void execute() {
    pivotSubsystem.runTilt(0.15);
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(pivotSubsystem.getVelocity()) > Shooter.MIN_TILT_RPM) {
      moving = true;
    }
    if (
      moving && Math.abs(pivotSubsystem.getVelocity()) < Shooter.MIN_TILT_RPM
    ) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      pivotSubsystem.resetTiltEncoder();
    }
    pivotSubsystem.runTilt(0);
    moving = false;
  }
}
