package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // Initialize the left and right climber motors and encoders
  CANSparkMax leftClimber = new CANSparkMax(
    15,
    CANSparkLowLevel.MotorType.kBrushless
  );
  CANSparkMax rightClimber = new CANSparkMax(
    16,
    CANSparkLowLevel.MotorType.kBrushless
  );

  RelativeEncoder leftEncoder = leftClimber.getEncoder();
  RelativeEncoder rightEncoder = rightClimber.getEncoder();

  /**
   * Constructor for the ClimberSubsystem class.
   *
   * The constructor for the ClimberSubsystem class sets the idle mode for the left and right climber motors to brake mode.
   */
  public ClimberSubsystem() {
    // Set the idle mode for the left and right climber motors to brake mode by default
    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Method to run the climbers at a given speed.
   *
   * @param speed The speed to run the climbers at.
   * @param isBreakMode Whether or not to run the climbers in brake mode.
   */
  public void runClimbers(double speed, boolean isBreakMode) {
    // Run the left and right climber motors at the given speed
    leftClimber.set(-speed);
    rightClimber.set(speed);
    // Set the idle mode for the left and right climber motors to brake mode if isBreakMode is true, else set the idle mode to coast mode
    if (isBreakMode) {
      leftClimber.setIdleMode(IdleMode.kBrake);
      rightClimber.setIdleMode(IdleMode.kBrake);
    } else {
      leftClimber.setIdleMode(IdleMode.kCoast);
      rightClimber.setIdleMode(IdleMode.kCoast);
    }
    // Put the left and right climber encoder values to the SmartDashboard for debugging
    SmartDashboard.putNumber("Right Climber Encoder", getEncoders()[0]);
    SmartDashboard.putNumber("Left Encoder", getEncoders()[1]);
    leftClimber.setSmartCurrentLimit(20, 80);
    rightClimber.setSmartCurrentLimit(20, 80);
  }

  /**
   * Method to reset the encoders on the climbers.
   */
  public void resetEncoders() {
    // Reset the left and right climber encoder positions to 0
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  /**
   * Method to get the encoders on the climbers.
   *
   * @return The encoders on the climbers.
   */
  public double[] getEncoders() {
    // Return the left and right climber encoder positions
    double[] result = { rightEncoder.getPosition(), leftEncoder.getPosition() };

    return result;
  }

  /**
   * Method to put data to the SmartDashboard for debugging
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right Climber Encoder", getEncoders()[0]);
    SmartDashboard.putNumber("Left Climber Encoder", getEncoders()[1]);
    SmartDashboard.putNumber(
      "Right Climber Current",
      rightClimber.getOutputCurrent()
    );
    SmartDashboard.putNumber(
      "Left Climber Current",
      leftClimber.getOutputCurrent()
    );
  }
}
