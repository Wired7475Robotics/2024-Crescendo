package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command{
    IntakeSubsystem intake;
    public IntakeCommand(IntakeSubsystem IntakeSubsystem){
        addRequirements(IntakeSubsystem);
        intake = IntakeSubsystem;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
