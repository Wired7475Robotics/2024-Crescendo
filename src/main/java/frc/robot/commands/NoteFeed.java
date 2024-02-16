package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.RollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class NoteFeed extends Command{
    RollerSubsystem shooter;
    IntakeSubsystem intake;
    public NoteFeed(RollerSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem){
        addRequirements(shooterSubsystem);
        shooter = shooterSubsystem;
        intake = intakeSubsystem;
    }
    @Override
    public void execute() {
        shooter.runRollers(true, -1);
        intake.runIntakeRollers(-1);
    }
    @Override
    public void end(boolean interrupted) {
        shooter.runRollers(true, 0);
        intake.runIntakeRollers(0);
    }
}
