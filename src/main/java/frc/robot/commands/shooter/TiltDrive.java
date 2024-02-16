package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.TiltSubsystem;

public class TiltDrive extends Command {

  PIDController tiltPID;
  TiltSubsystem tiltDrive;
  XboxController ControllerAxis;

  public TiltDrive(
    TiltSubsystem tiltSubsystem,
    XboxController xboxControllerAxis
  ) {
    tiltDrive = tiltSubsystem;
    ControllerAxis = xboxControllerAxis;
    addRequirements(tiltSubsystem);
    tiltPID = Constants.Shooter.tiltPIDcontrol;
  }

  @Override
  public void execute() {
    tiltPID.setSetpoint(tiltDrive.getTargetAngle(ControllerAxis.getLeftY()));
    tiltDrive.runTilt(-tiltPID.calculate(tiltDrive.getTiltDegrees()));
  }

  @Override
  public void end(boolean interrupted) {
    tiltDrive.runTilt(0);
  }
}
