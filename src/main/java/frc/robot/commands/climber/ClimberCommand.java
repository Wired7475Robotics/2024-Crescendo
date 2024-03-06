package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCommand extends Command {

  // Initialize the ClimberSubsystem and XboxController variables
  private ClimberSubsystem climber;
  private XboxController controller;

  /**
   * Constructor for the ClimberCommand class.
   *
   * The constructor for the ClimberCommand class sets the ClimberSubsystem and XboxController variables.
   *
   * @param climberSubsystem The ClimberSubsystem to use.
   * @param controller The XboxController to use.
   */
  public ClimberCommand(
    ClimberSubsystem climberSubsystem,
    XboxController controller
  ) {
    // Add the ClimberSubsystem to the requirements of the command and set the ClimberSubsystem and XboxController variables
    addRequirements(climberSubsystem);
    climber = climberSubsystem;
    this.controller = controller;
  }

  /**
   * Method to run the Climber motors at the speed of the left and right triggers on the XboxController.
   */
  @Override
  public void execute() {
    // Run the Climber motors at the speed of the left and right triggers on the XboxController
    double speed =
      controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
    climber.runClimbers(speed, true);
  }

  /**
   * Method to stop the Climber motors.
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the Climber motors
    climber.runClimbers(0, false);
  }
}
