package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class DeployerSubsystem extends SubsystemBase {

  // The target value for the Deployer motor
  double targetValue;
  // The conversion factor for the Deployer motor
  double deployConversionFactor;
  //Motor for deploying intake
  CANSparkMax deployer = new CANSparkMax(
    9,
    CANSparkLowLevel.MotorType.kBrushless
  );

  /**
   * Constructor for the DeployerSubsystem class.
   *
   * The constructor for the DeployerSubsystem class initializes the DeployerSubsystem's motor and sets the conversion factor for the motor.
   */
  public DeployerSubsystem() {
    // Set the conversion factor for the Deployer motor to the value in Constants. This is done to save processing power later by avoiding redoing the math every time we need to get the conversion factor
    deployConversionFactor = Constants.Intake.INTAKE_CONVERSION_FACTOR;
    // Set the idle mode for the Deployer motor to brake
    deployer.setIdleMode(IdleMode.kBrake);
    // Set the current limit for the Deployer motor to 50
    deployer.setSmartCurrentLimit(50);
  }

  /**
   * Method to run the Deployer motor at a given speed.
   *
   * @param speed The speed to run the Deployer motor at.
   */
  public void runDeployer(double speed) {
    // Run the deployer motor at the given speed, as long as that speed is within the bounds of the max intake speed
    deployer.set(
      Math.min(
        Constants.Intake.INTAKE_SPEED,
        Math.max(-Constants.Intake.INTAKE_SPEED, speed)
      )
    );

    System.out.println(speed);
  }

  /**
   * Method to run the Deployer motor at a slow speed.
   */
  public void runSlow() {
    // Run the deployer motor at a slow speed
    runDeployer(Intake.SLOW_SPEED);
  }

  /**
   * Method to get the velocity of the deploy motor.
   * @return The velocity of the deploy motor in RPM.
   */
  public double getVelocity() {
    // Return the velocity of the deploy motor
    return deployer.getEncoder().getVelocity();
  }

  /**
   * Method to put data to the SmartDashboard for debugging
   */
  @Override
  public void periodic() {
    // Call the superclass's periodic method
    super.periodic();
    // Put the deployer motor's current to the SmartDashboard
    SmartDashboard.putNumber("DeployerCurrent", deployer.getOutputCurrent());
    SmartDashboard.putNumber("IntakeVelocity", getVelocity());
  }
}
