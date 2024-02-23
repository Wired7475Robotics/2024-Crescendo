package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimbCommand extends Command {

  private ClimberSubsystem climber;
  private XboxController controller;

  public ClimbCommand(
    ClimberSubsystem climberSubsystem,
    XboxController controller
  ) {
    addRequirements(climberSubsystem);
    climber = climberSubsystem;
    this.controller = controller;
  }

  @Override
  public void execute() {
    double speed =
      controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
    climber.runClimbers(speed, true);
  }

  @Override
  public void end(boolean interrupted) {
    climber.runClimbers(0, false);
  }
}
