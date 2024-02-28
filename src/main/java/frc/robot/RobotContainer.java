// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Intake;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.climber.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.DeployerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.RollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TiltSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import javax.swing.text.html.HTMLDocument.HTMLReader.HiddenAction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );
  private static final ShooterSubsystem shooter = new ShooterSubsystem();

  private static final RollerSubsystem rollers = new RollerSubsystem();

  private static final IntakeSubsystem intake = new IntakeSubsystem();

  private static final ClimberSubsystem climber = new ClimberSubsystem();

  private static final TiltSubsystem tiltDrive = new TiltSubsystem();

  private static final DeployerSubsystem intakeDeployer = new DeployerSubsystem();

  Joystick leftDriverNunchuck = new Joystick(0);
  Joystick rightDriverNunchuck = new Joystick(1);

  XboxController operatorXbox = new XboxController(2);

  int noteStatus = OperatorConstants.FALSE;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    climber.resetEncoders();
    tiltDrive.resetTiltEncoder();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(
      drivebase,
      // Applies deadbands and inverts controls because joysticks
      // are back-right positive while robot
      // controls are front-left positive
      () ->
        MathUtil.applyDeadband(
          -drivebase.getAxis(
            leftDriverNunchuck.getY(),
            leftDriverNunchuck.getRawButton(1),
            rightDriverNunchuck.getRawButton(1)
          ),
          OperatorConstants.LEFT_Y_DEADBAND
        ),
      () ->
        MathUtil.applyDeadband(
          -drivebase.getAxis(
            leftDriverNunchuck.getX(),
            leftDriverNunchuck.getRawButton(1),
            rightDriverNunchuck.getRawButton(1)
          ),
          OperatorConstants.LEFT_X_DEADBAND
        ),
      () -> -rightDriverNunchuck.getX(),
      () -> -rightDriverNunchuck.getY()
    );

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(
      drivebase,
      () ->
        MathUtil.applyDeadband(
          drivebase.getAxis(
            leftDriverNunchuck.getY(),
            leftDriverNunchuck.getRawButton(1),
            rightDriverNunchuck.getRawButton(1)
          ),
          OperatorConstants.LEFT_Y_DEADBAND
        ),
      () ->
        MathUtil.applyDeadband(
          drivebase.getAxis(
            leftDriverNunchuck.getX(),
            leftDriverNunchuck.getRawButton(1),
            rightDriverNunchuck.getRawButton(1)
          ),
          OperatorConstants.LEFT_X_DEADBAND
        ),
      () -> rightDriverNunchuck.getY()
    );

    drivebase.setDefaultCommand(
      !RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive
    );

    TiltDrive tiltCommand = new TiltDrive(tiltDrive);
    ClimbCommand climbCommand = new ClimbCommand(climber, operatorXbox);
    tiltDrive.setDefaultCommand(tiltCommand);
    climber.setDefaultCommand(climbCommand);
    intakeDeployer.setDefaultCommand(
      new InstantCommand(intakeDeployer::runTiltSlow, intakeDeployer)
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    RunRollerCommand rollerCommand = new RunRollerCommand(intake, false, -0.75);
    SequentialCommandGroup intakeCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> noteStatus = OperatorConstants.NULL),
      new ParallelCommandGroup(
        new IntakeDeployCommand(intakeDeployer, Intake.LOW),
        rollerCommand,
        new IndexerCommand(rollers, false, 0.75)
          .until(rollerCommand::isFinished)
      ),
      new ParallelCommandGroup(
        new IntakeDeployCommand(intakeDeployer, Intake.HIGH),
        new IndexerCommand(rollers, false, 0.15)
      ),
      new IndexerCommand(rollers, true, -0.3).withTimeout(0.2),
      new InstantCommand(() -> noteStatus = OperatorConstants.TRUE)
    );
    SequentialCommandGroup cancelIntake = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new IndexerCommand(rollers, true, 0).withTimeout(0.1),
        new RunRollerCommand(intake, true, 0).withTimeout(0.1),
        new InstantCommand(() -> noteStatus = OperatorConstants.FALSE)
      ),
      new IntakeDeployCommand(intakeDeployer, Intake.HIGH)
    );

    ParallelCommandGroup fire = new ParallelCommandGroup(
      new RunShooterTeleop(shooter),
      new WaitCommand(2)
        .andThen(new IndexerCommand(rollers, false, 1)),
      new InstantCommand(() -> noteStatus = OperatorConstants.FALSE)
    );



    new JoystickButton(rightDriverNunchuck, 2)
      .onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(operatorXbox, 1)
      .toggleOnTrue(
        intakeCommandGroup.asProxy().finallyDo(() ->
          CommandScheduler.getInstance().schedule(cancelIntake)
        )
      );

    new JoystickButton(operatorXbox, 3).toggleOnTrue(fire);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Debug Path", true);
  }

  public void setDriveMode() {
    drivebase.setDefaultCommand(drivebase.getDefaultCommand());
  }

  public void setShooterCommand() {
    //shooter.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void calibrateCameraAngle(){
    SmartDashboard.putNumber("Calibration Angle",tiltDrive.calibrateCameraAngle(LimelightHelpers.getTY("")));
  }
}
