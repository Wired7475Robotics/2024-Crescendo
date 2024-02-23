package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.DeployerSubsystem;

public class IntakeDeployCommand extends Command {

  PIDController tiltPID;
  DeployerSubsystem tiltDrive;
  XboxController ControllerAxis;

  public IntakeDeployCommand(
    DeployerSubsystem DeployerSubsystem,
    XboxController xboxControllerAxis
  ) {
    tiltDrive = DeployerSubsystem;
    ControllerAxis = xboxControllerAxis;
    addRequirements(DeployerSubsystem);
    tiltPID = Constants.Intake.tiltPIDcontrol;
  }

  @Override
  public void execute() {
    tiltPID.setSetpoint(tiltDrive.getTargetAngle(ControllerAxis.getRightY()));
    tiltDrive.runTilt(tiltPID.calculate(tiltDrive.getTiltDegrees()));
  }

  @Override
  public void end(boolean interrupted) {
    tiltDrive.runTilt(0);
  }
}
