package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.Constants.Shooter.MathConstants;
import frc.robot.subsystems.shooter.PivotSubsystem;

public class PivotCommand extends Command {

  PIDController pivotPidController;
  PivotSubsystem pivotSubsystem;
  XboxController ControllerAxis;
  double target;
  boolean useCamera = true;
  DoubleSupplier m_xDist;
  DoubleSupplier m_yDist;

  /**
   * Constructor for the PivotCommand class.
   *
   * @param tiltSubsystem The tilt subsystem to use.
   */
  public PivotCommand(
    PivotSubsystem tiltSubsystem,
    DoubleSupplier xDist,
    DoubleSupplier yDist
  ) {
    // set the pivotSubsystem to the tiltSubsystem
    this.pivotSubsystem = tiltSubsystem;
    // add the tiltSubsystem to the requirements
    addRequirements(tiltSubsystem);
    // set the pivotPidController to the pivotPidControllercontrol from the Constants file
    pivotPidController = Constants.Shooter.pivotPidController;
    // set the useCamera to true
    m_xDist = xDist;
    m_yDist = yDist;
  }

  public PivotCommand(PivotSubsystem tiltSubsystem, double targetAngle) {
    // set the pivotSubsystem to the tiltSubsystem
    this.pivotSubsystem = tiltSubsystem;
    // add the tiltSubsystem to the requirements
    addRequirements(tiltSubsystem);
    // set the pivotPidController to the pivotPidControllercontrol from the Constants file
    pivotPidController = Constants.Shooter.pivotPidController;
    target = targetAngle;
    // set the useCamera to false
    useCamera = false;
  }

  /**
   * Method to execute the command.
   *
   * This method runs the pivotPidController to calculate the target angle and then runs the tilt motor at the calculated speed.
   */
  @Override
  public void execute() {
    // if useCamera is true then set the target to the calculated target angle from the Limelight
    if (useCamera) {
      // calculate the target angle from the pivotSubsystem
      target = Math.toDegrees(Math.atan2(m_yDist.getAsDouble() + Math.pow(m_xDist.getAsDouble(), MathConstants.GRAVITY_CONSTANT),m_xDist.getAsDouble()));
    }
    // set the pivotPidController setpoint to the target
    pivotPidController.setSetpoint(target);
    // run the tilt motor at the calculated speed
    pivotSubsystem.runTilt(
      -pivotPidController.calculate(pivotSubsystem.getTiltDegrees())
    );
  }

  /**
   * Method to end the command.
   *
   * This method stops the tilt motor when the command ends.
   */
  @Override
  public void end(boolean interrupted) {
    // stop the tilt motor
    pivotSubsystem.runTilt(0);
  }
}
