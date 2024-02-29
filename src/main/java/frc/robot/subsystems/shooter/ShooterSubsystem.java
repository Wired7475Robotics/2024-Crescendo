package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {

  // Motor driving the roller-feed

  // Initialize the shooter motors
  CANSparkMax shooterLeft = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax shooterRight = new CANSparkMax(14, MotorType.kBrushless);

  /**
   * Constructor for the ShooterSubsystem class.
   *
   * Initializes the shooter motors and sets the ramp rate for the motors.
   */
  public ShooterSubsystem() {
    shooterLeft.setOpenLoopRampRate(2);
    shooterRight.setOpenLoopRampRate(2);
  }

  /**
   * Method to run the shooter motors at a given speed.
   *
   * @param speed The speed to run the shooter motors at.
   * @param isBreakMode Whether or not to run the shooter motors in break mode.
   * @param diff The difference between the two shooter motors.
   */
  public void runShooter(double speed, boolean isBreakMode, double diff) {
    // Run the shooter motors at the given speed
    shooterLeft.set(speed);
    shooterRight.set(-speed + diff);
    // Set the shooter motors to brake mode if isBreakMode is true
    if (isBreakMode) {
      shooterLeft.setIdleMode(IdleMode.kBrake);
      shooterRight.setIdleMode(IdleMode.kBrake);
    } else {
      // Set the shooter motors to coast mode if isBreakMode is false
      shooterLeft.setIdleMode(IdleMode.kCoast);
      shooterRight.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Method to check if the shooter motors are ready to fire.
   *
   * checks shooter rpm to see if it is ready to fire
   * @return
   */
  public boolean isReady() {
    // Return true if the shooter motors are ready to fire
    return (
        shooterLeft.getEncoder().getVelocity() >
        Shooter.MIN_SHOOTING_RPM *
        shooterLeft.get() &&
        shooterRight.getEncoder().getVelocity() >
        Shooter.MIN_SHOOTING_RPM *
        shooterRight.get()
      )
      ? true
      : false;
  }
}
