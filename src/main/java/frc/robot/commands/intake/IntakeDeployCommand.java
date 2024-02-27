package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.DeployerSubsystem;

public class IntakeDeployCommand extends Command {

  PIDController tiltPID;
  DeployerSubsystem tiltDrive;
  int pos;
  boolean moving;

  public IntakeDeployCommand(
    DeployerSubsystem DeployerSubsystem,
    int position
  ) {
    tiltDrive = DeployerSubsystem;
    addRequirements(DeployerSubsystem);
    tiltPID = Constants.Intake.tiltPIDcontrol;
    pos = position;
  }

  @Override
  public void execute() {
    if (pos == 1){
      tiltDrive.runTilt(-0.5);
    } else {
      tiltDrive.runTilt(0.3);
    }
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(tiltDrive.getVelocity())
     > 100){
      moving = true;
    } 
    if (moving && Math.abs(tiltDrive.getVelocity()) < 100){
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    tiltDrive.runTilt(0);
    moving = false;
  }
}
