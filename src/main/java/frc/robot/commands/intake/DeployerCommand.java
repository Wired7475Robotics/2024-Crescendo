package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.intake.DeployerSubsystem;

public class DeployerCommand extends Command {

  // Initialize the DeployerSubsystem position and moving variables
  DeployerSubsystem deployer;
  int pos;
  boolean moving;

  /**
   * Constructor for the DeployerCommand class.
   *
   * The constructor for the DeployerCommand class sets the DeployerSubsystem and position variables.
   *
   * @param deployerSubsystem The DeployerSubsystem to use.
   * @param position The position to set the DeployerSubsystem to.
   */
  public DeployerCommand(DeployerSubsystem deployerSubsystem, int position) {
    deployer = deployerSubsystem;
    addRequirements(deployerSubsystem);
    pos = position;
  }

  /**
   * Method to run the Deployer motor at a given speed.
   */
  @Override
  public void execute() {
    // if the position is high, run the deployer motor at a higher speed to overcome gravity, else run the deployer motor at a lower speed and reverse
    if (pos == Intake.HIGH) {
      deployer.runDeployer(Intake.RAISE_SPEED);
    } else {
      deployer.runDeployer(Intake.LOWER_SPEED);
    }
  }

  /**
   * Method to check if the Deployer motor is finished moving.
   *
   * @return Whether or not the Deployer motor is finished moving.
   */
  @Override
  public boolean isFinished() {
    // return true if the deployer motor is moving and then stops moving
    if (Math.abs(deployer.getVelocity()) > Intake.MIN_SPEED) {
      moving = true;
    }
    if (moving && Math.abs(deployer.getVelocity()) < Intake.MIN_SPEED) {
      return true;
    }
    return false;
  }

  /**
   * Method to stop the Deployer motor.
   *
   * @param interrupted Whether or not the command was interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    // stop the deployer motor and set moving to false
    deployer.runDeployer(0);
    moving = false;
  }
}
