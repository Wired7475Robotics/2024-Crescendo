package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimbCommand extends Command {

    private ClimberSubsystem climber;

    public ClimbCommand(ClimberSubsystem climberSubsystem){
        addRequirements(climberSubsystem);
        climber = climberSubsystem;
    }

    @Override
    public void initialize() {
        climber.runClimbers(0.75, true);
        try {
            wait(50);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        climber.runClimbers(0, true);
    }

    @Override
    public void execute() {
        climber.runClimbers(0, false);
        try {
            wait(50);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        climber.runClimbers(0.75, true);
        try {
            wait(50);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        climber.runClimbers(0, false);
    }
}
