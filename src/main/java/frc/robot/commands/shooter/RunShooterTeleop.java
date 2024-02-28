package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooterTeleop extends Command {

  ShooterSubsystem shooter;

  public RunShooterTeleop(ShooterSubsystem ShooterSubsytem) {
    addRequirements(ShooterSubsytem);
    shooter = ShooterSubsytem;
  }

  @Override
  public void execute() {
    shooter.runShooter(1, false, 0.1);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runShooter(0, false, 0);
  }
}
