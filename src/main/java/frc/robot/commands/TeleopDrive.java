package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controll;
import frc.robot.Robot;

public class TeleopDrive extends Command{
    double leftStickX;
    double leftStickY;
    double rightStickX;
    double rightStickY;
    double targetAngle;
    double currentAngle;
    double lastAngle = 0;

    final double FASTCOEF = 2;
    final double SLOWCOEF = 0.5;
    final double DEGREEINCREMEMNT = 0.1;
    final double DEADZONE = 0.9;
    

    public TeleopDrive(){
        addRequirements(Robot.drive);
    }
    
    public static double mod(double a, double b) { //booleg modulo function, works same as in python
		double c = a % b;
		c = c < 0? c + b : c;
		return c;
		}

    @Override
    public void initialize() {
        //Set PID values and enable PID
    }
    @Override
    public void execute() {
        //System.out.println("Exectuting drive command");
        //Get left stick values and calculate target angle
        leftStickY = -Robot.controlers.getDriveLeftStick(Controll.Y);
        leftStickX = -Robot.controlers.getDriveLeftStick(Controll.X);
        
        
        currentAngle = mod(Robot.navx.getAngle(), 360); //modulo starting angle
        targetAngle = Math.round(mod(Math.toDegrees(Math.atan2(leftStickY,leftStickX)) - 90, 360)/DEGREEINCREMEMNT)*DEGREEINCREMEMNT; //get target angle, between 0 and 360 deg

        if (Math.abs(leftStickY) < DEADZONE && Math.abs(leftStickX) < DEADZONE) { //prevent turning when in deadzone
			targetAngle = lastAngle;
		} else {
            lastAngle = targetAngle;
        }

        //Calculate shortest distance to target angle
		double diff = currentAngle - targetAngle;
		diff = mod( (diff + 180) , 360 ) - 180; //equals between -180 and 180
        
        //Send calculated rotation velocity to driveTrain
        Robot.drive.setRotationVelocity((diff/180)*16); //should equal between -4 and 4 (for now)
        
        //print info
        System.out.println("Target Angle: "+ targetAngle + " Current Angle: " + currentAngle + " Difference: " + diff + " Output: " + (diff/180));

        rightStickX = Robot.controlers.getDriveRightStick(Controll.X);
        rightStickY = Robot.controlers.getDriveRightStick(Controll.Y);
        
        //Add a fast/Slow mode that depends on button presses
        rightStickY = Controll.getDriveBumper(Controll.RIGHT)? rightStickY * FASTCOEF : Controll.getDriveBumper(Controll.LEFT)? rightStickY * SLOWCOEF : rightStickY;
        rightStickX = Controll.getDriveBumper(Controll.RIGHT)? rightStickX * FASTCOEF : Controll.getDriveBumper(Controll.LEFT)? rightStickX * SLOWCOEF : rightStickX;
        
        System.out.println("Right Stick: " + rightStickX + "," +  rightStickY);

        Robot.drive.setTranslationVelocity(rightStickX, rightStickY);
    }
}
