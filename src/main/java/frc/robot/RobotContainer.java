// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Shooter;
import frc.robot.Util.FieldElements;
import frc.robot.commands.climber.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.lang.reflect.Field;

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

  private static final IndexerSubsystem indexer = new IndexerSubsystem();

  private static final IntakeSubsystem intake = new IntakeSubsystem();

  private static final ClimberSubsystem climber = new ClimberSubsystem();

  private static final PivotSubsystem pivot = new PivotSubsystem();

  private static final DeployerSubsystem intakeDeployer = new DeployerSubsystem();

  Joystick leftDriverNunchuck = new Joystick(0);
  Joystick rightDriverNunchuck = new Joystick(1);

  XboxController operatorXbox = new XboxController(2);

  int noteStatus = OperatorConstants.FALSE;

  Timer pivotTimer = new Timer();
  Timer driveTimer = new Timer();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.zeroGyro();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
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
      () -> rightDriverNunchuck.getX(),
      () -> rightDriverNunchuck.getY()
    );

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
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
      () -> rightDriverNunchuck.getRawAxis(2)
    );

    drivebase.setDefaultCommand(
      !RobotBase.isSimulation()
        ? driveFieldOrientedDirectAngle
        : driveFieldOrientedDirectAngleSim
    );

    drivebase.getRelativeInterpolatedPosition(
      pivotTimer,
      0.1,
      FieldElements.kSpeakerCenterBlue,
      FieldElements.kSpeakerCenterRed
    );
    PivotCommand tiltCommand = new PivotCommand(pivot, Shooter.MAX_TILT);
    ClimberCommand climbCommand = new ClimberCommand(climber, operatorXbox);
    pivot.setDefaultCommand(tiltCommand);
    climber.setDefaultCommand(climbCommand);
    intakeDeployer.setDefaultCommand(
      new InstantCommand(intakeDeployer::runSlow, intakeDeployer)
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

    climber.resetEncoders();
    pivot.resetTiltEncoder();

    new JoystickButton(rightDriverNunchuck, 2)
      .onTrue((Commands.runOnce(drivebase::zeroGyro)));

    // Setup command sequences

    // Command sequence for intaking and storeing note
    SequentialCommandGroup intakeCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> noteStatus = OperatorConstants.NULL),
      // Deploy intake and start intake and indexer
      new ParallelCommandGroup(
        new DeployerCommand(intakeDeployer, Intake.LOW),
        new ParallelRaceGroup(
          new IntakeCommand(intake, false, -0.75),
          new IndexerCommand(indexer, false, 0.75)
        )
      ),
      // Stow intake and stop intake and slow indexer
      new ParallelCommandGroup(
        new DeployerCommand(intakeDeployer, Intake.HIGH),
        new IndexerCommand(indexer, false, 0.15)
      ),
      // run indexer backwardsand set the status to true to tell the robot that the note is stored
      new IndexerCommand(indexer, true, -0.3).withTimeout(0.2),
      new InstantCommand(() -> noteStatus = OperatorConstants.TRUE)
    );

    // Command sequence for canceling intake command
    SequentialCommandGroup cancelIntake = new SequentialCommandGroup(
      // Stop intake and stop indexer and tell the robot that the note is not stored
      new ParallelCommandGroup(
        new IndexerCommand(indexer, true, 0).withTimeout(0.1),
        new IntakeCommand(intake, true, 0).withTimeout(0.1),
        new InstantCommand(() -> noteStatus = OperatorConstants.FALSE)
      ),
      // Stow intake
      new DeployerCommand(intakeDeployer, Intake.HIGH)
    );

    // Command sequence for firing the note
    ParallelCommandGroup fire = new ParallelCommandGroup(
      new ParallelRaceGroup(
        // run shooter and wait until shooter is ready and robot is aimed at the target then run indexer

        new PivotCommand(
          pivot,
          () ->
            drivebase
              .getRelativeInterpolatedPosition(
                pivotTimer,
                Shooter.AIMING_TIME,
                FieldElements.kSpeakerCenterRed,
                FieldElements.kSpeakerCenterBlue
              )
              .getX(),
          () ->
            drivebase
              .getRelativeInterpolatedPosition(
                pivotTimer,
                Shooter.AIMING_TIME,
                FieldElements.kSpeakerCenterRed,
                FieldElements.kSpeakerCenterBlue
              )
              .getY()
        ),
        new WaitUntilCommand(() -> isReady())
          .andThen(new IndexerCommand(indexer, true, 1))
          .andThen(new WaitCommand(0.5))
          .andThen(
            new InstantCommand(() -> noteStatus = OperatorConstants.FALSE)
          ),
        new InstantCommand(driveTimer::start)
          .andThen(
            new AbsoluteFieldDrive(
              drivebase,
              // Applies deadbands and inverts controls because joysticks
              // are back-right positive while robot
              // controls are front-left positive
              (
                () ->
                  -drivebase.getAxis(
                    leftDriverNunchuck.getY(),
                    leftDriverNunchuck.getRawButton(1),
                    rightDriverNunchuck.getRawButton(1)
                  )
              ),
              (
                () ->
                  -drivebase.getAxis(
                    leftDriverNunchuck.getX(),
                    leftDriverNunchuck.getRawButton(1),
                    rightDriverNunchuck.getRawButton(1)
                  )
              ),
              (
                () ->
                  drivebase.getTargetAngle(
                    Math.toDegrees(
                      Math.atan2(
                        drivebase
                          .getRelativeInterpolatedPosition(
                            driveTimer,
                            Drivebase.AIMING_TIME,
                            FieldElements.kSpeakerCenterRed,
                            FieldElements.kSpeakerCenterBlue
                          )
                          .getX(),
                        drivebase
                          .getRelativeInterpolatedPosition(
                            driveTimer,
                            Drivebase.AIMING_TIME,
                            FieldElements.kSpeakerCenterRed,
                            FieldElements.kSpeakerCenterBlue
                          )
                          .getZ()
                      )
                    )
                  )
              )
            )
          )
          .andThen(new InstantCommand(driveTimer::reset))
          .andThen(new InstantCommand(pivotTimer::reset))
      ),
      // tell the robot that the note is not stored
      new InstantCommand(() -> noteStatus = OperatorConstants.FALSE)
    );

    // bind command sequences to buttons

    // if the note is not stored, run intake command sequence. if the intake command sequence is running, cancel the intake command sequence
    new JoystickButton(operatorXbox, 1)
      .toggleOnTrue(
        intakeCommandGroup
          .onlyIf(() -> noteStatus == OperatorConstants.FALSE)
          .asProxy()
          .finallyDo(() ->
            CommandScheduler
              .getInstance()
              .schedule(
                cancelIntake.onlyIf(() -> noteStatus == OperatorConstants.NULL)
              )
          )
      );
    // if the note is stored, run fire command sequence
    new JoystickButton(operatorXbox, 3)
      .whileTrue(fire.onlyIf(() -> noteStatus == OperatorConstants.TRUE));
    new JoystickButton(rightDriverNunchuck, 2)
      .onTrue(new InstantCommand(drivebase::zeroGyro));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setShooterCommand() {
    shooter.setDefaultCommand(
      new RepeatCommand(
        new ShooterCommand(shooter, -1, -0.325)
          .onlyIf(() -> noteStatus == OperatorConstants.TRUE)
      )
    );
  }

  public boolean isReady() {
    return drivebase.isReady() && pivot.isReady();
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
