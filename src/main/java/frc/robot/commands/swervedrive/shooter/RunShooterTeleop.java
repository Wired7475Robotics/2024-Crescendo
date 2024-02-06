package frc.robot.commands.swervedrive.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooter extends Command{
    ShooterSubsystem shooter;
    public RunShooter(ShooterSubsystem ShooterSubsytem){
        addRequirements(ShooterSubsytem);
        shooter = ShooterSubsytem;
    }

    @Override
    public void execute() {
        shooter.runShooter(0.9);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.runShooter(0);
        Robot
    }
}
