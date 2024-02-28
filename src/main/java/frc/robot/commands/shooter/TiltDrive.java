package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.shooter.TiltSubsystem;

public class TiltDrive extends Command {

  PIDController tiltPID;
  TiltSubsystem tiltDrive;
  XboxController ControllerAxis;
  double target;
  boolean useCamera = true;

  public TiltDrive(
    TiltSubsystem tiltSubsystem
  ) {
    tiltDrive = tiltSubsystem;
    addRequirements(tiltSubsystem);
    tiltPID = Constants.Shooter.tiltPIDcontrol;
  }

  public TiltDrive(TiltSubsystem tiltSubsystem, double targetAngle){
    tiltDrive = tiltSubsystem;
    addRequirements(tiltSubsystem);
    tiltPID = Constants.Shooter.tiltPIDcontrol;
    target = targetAngle;
    useCamera = false;
  }

  @Override
  public void execute() {
    if (useCamera){
      double aprilTagAngle = LimelightHelpers.getTY("");
      target = tiltDrive.calculateTargetAngle(aprilTagAngle);
      
    }
    tiltPID.setSetpoint(target);
    tiltDrive.runTilt(-tiltPID.calculate(tiltDrive.getTiltDegrees()));
    
  }

  @Override
  public void end(boolean interrupted) {
    tiltDrive.runTilt(0);
  }
}
