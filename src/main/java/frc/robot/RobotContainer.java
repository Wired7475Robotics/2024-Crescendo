// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  int step = 0;

  private static final ShooterSubsystem shooter = new ShooterSubsystem();

  private static final IndexerSubsystem indexer = new IndexerSubsystem();

  private static final IntakeSubsystem intake = new IntakeSubsystem();

  private static final ClimberSubsystem climber = new ClimberSubsystem();

  private static final PivotSubsystem pivot = new PivotSubsystem();

  private static final DeployerSubsystem intakeDeployer = new DeployerSubsystem();

  public static final Joystick leftDriverNunchuck = new Joystick(0);
  public static final Joystick rightDriverNunchuck = new Joystick(1);
  public static final String RedAliance = "red";
  public static final String BlueAliance = "blue";

  public static final String twoNoteMid = "2 Note Mid";
  public static final String twoNoteLeft = "2 Note Left";
  public static final String threeNoteMid = "3 Note Mid";
  public static final String threeNoteLeft = "3 Note Left";
  public static final String driveOut = "Simple Drive";
  public static final String oneNote = "1 Note";

  public static SendableChooser<String> alianceChooser = new SendableChooser<>();
  public static SendableChooser<String> autonChooser = new SendableChooser<>();

  XboxController operatorXbox = new XboxController(2);

  boolean autoFire = false;

  int noteStatus = OperatorConstants.TRUE;

  Timer pivotTimer = new Timer();
  Timer driveTimer = new Timer();

  boolean overideControls = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    alianceChooser.setDefaultOption("Blue", BlueAliance);
    alianceChooser.addOption("Red", RedAliance);
    autonChooser.setDefaultOption(twoNoteMid, twoNoteMid);
    autonChooser.addOption(driveOut, driveOut);
    autonChooser.addOption(twoNoteLeft, twoNoteLeft);
    autonChooser.addOption(threeNoteLeft, threeNoteLeft);
    autonChooser.addOption(threeNoteMid, threeNoteMid);
    autonChooser.addOption(oneNote, oneNote);
    SmartDashboard.putData(alianceChooser);
    SmartDashboard.putData(autonChooser);
    SmartDashboard.putNumber("Add Offset", 0);
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("Swerve Subsystem", drivebase);
    drivebase.zeroGyro();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () ->
        MathUtil.applyDeadband(
          -drivebase.getAxis(
            leftDriverNunchuck.getY(),
            leftDriverNunchuck.getRawButton(1),
            rightDriverNunchuck.getRawButton(1)
          ) *
          (alianceChooser.getSelected().equals(RedAliance) ? -1 : 1),
          OperatorConstants.LEFT_Y_DEADBAND
        ),
      () ->
        MathUtil.applyDeadband(
          -drivebase.getAxis(
            leftDriverNunchuck.getX(),
            leftDriverNunchuck.getRawButton(1),
            rightDriverNunchuck.getRawButton(1)
          ) *
          (alianceChooser.getSelected().equals(RedAliance) ? -1 : 1),
          OperatorConstants.LEFT_X_DEADBAND
        ),
      () ->
        -rightDriverNunchuck.getX() *
        (alianceChooser.getSelected().equals(RedAliance) ? -1 : 1),
      () ->
        -rightDriverNunchuck.getY() *
        (alianceChooser.getSelected().equals(RedAliance) ? -1 : 1)
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
    ClimberCommand climbCommand = new ClimberCommand(climber, operatorXbox);
    PivotCommand pivotCommand = new PivotCommand(pivot, 25);
    pivot.setDefaultCommand(pivotCommand);
    climber.setDefaultCommand(climbCommand);
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
    SmartDashboard.putNumber("Fine Angle", 0);
    SmartDashboard.putNumber("Raw Angle", 9);
    // Setup command sequences

    // Command sequence for intaking and storeing note
    SequentialCommandGroup intakeCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> noteStatus = OperatorConstants.NULL),
      // Deploy intake and start intake and indexer
      new ParallelCommandGroup(
        new DeployerCommand(intakeDeployer, Intake.LOW),
        new ParallelRaceGroup(
          new IntakeCommand(intake, false, -1),
          new IndexerCommand(indexer, false, 0.4)
        )
      ),
      // Stow intake and stop intake and slow indexer
      new ParallelCommandGroup(
        new DeployerCommand(intakeDeployer, Intake.HIGH),
        new IndexerCommand(indexer, false, 0.3)
      ),
      // run indexer backwardsand set the status to true to tell the robot that the note is stored
      new IndexerCommand(indexer, true, -0.4)
        .withTimeout(0.5)
        .raceWith(new ShooterCommand(shooter, 1, 0)),
      new InstantCommand(() -> noteStatus = OperatorConstants.TRUE)
    );

    // Command sequence for firing the note
    ParallelCommandGroup fire = new ParallelCommandGroup(
      new ParallelRaceGroup(
        new ShooterCommand(shooter, -1, -0.35),
        new WaitCommand(2.5)
          .andThen(new IndexerCommand(indexer, true, 1).withTimeout(0.75))
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
                  drivebase.getAxis(
                    leftDriverNunchuck.getY(),
                    leftDriverNunchuck.getRawButton(1),
                    rightDriverNunchuck.getRawButton(1)
                  ) *
                  (alianceChooser.getSelected().equals(RedAliance) ? 1 : -1)
              ),
              (
                () ->
                  drivebase.getAxis(
                    leftDriverNunchuck.getX(),
                    leftDriverNunchuck.getRawButton(1),
                    rightDriverNunchuck.getRawButton(1)
                  ) *
                  (alianceChooser.getSelected().equals(RedAliance) ? 1 : -1)
              ),
              (
                () ->
                  (
                    Math.toDegrees(
                      Math.atan(
                        -drivebase
                          .getRelativeInterpolatedPosition(
                            driveTimer,
                            Drivebase.AIMING_TIME,
                            FieldElements.kSpeakerCenterRed,
                            FieldElements.kSpeakerCenterBlue,
                            alianceChooser.getSelected().equals(RedAliance)
                          )
                          .getY() /
                        -drivebase
                          .getRelativeInterpolatedPosition(
                            driveTimer,
                            Drivebase.AIMING_TIME,
                            FieldElements.kSpeakerCenterRed,
                            FieldElements.kSpeakerCenterBlue,
                            alianceChooser.getSelected().equals(RedAliance)
                          )
                          .getX()
                      )
                    ) +
                    (alianceChooser.getSelected().equals(RedAliance) ? 180 : 0)
                  ) /
                  180
              )
            )
          )
          .andThen(new InstantCommand(driveTimer::reset))
          .andThen(new InstantCommand(pivotTimer::reset)),
        new PivotCommand(
          pivot,
          () ->
            Math.sqrt(
              Math.pow(
                -drivebase
                  .getRelativeInterpolatedPosition(
                    pivotTimer,
                    Shooter.AIMING_TIME,
                    FieldElements.kSpeakerCenterRed,
                    FieldElements.kSpeakerCenterBlue,
                    alianceChooser.getSelected().equals(RedAliance)
                  )
                  .getX(),
                2
              ) +
              Math.pow(
                -drivebase
                  .getRelativeInterpolatedPosition(
                    pivotTimer,
                    Shooter.AIMING_TIME,
                    FieldElements.kSpeakerCenterRed,
                    FieldElements.kSpeakerCenterBlue,
                    alianceChooser.getSelected().equals(RedAliance)
                  )
                  .getY(),
                2
              )
            ),
          () ->
            -drivebase
              .getRelativeInterpolatedPosition(
                pivotTimer,
                Shooter.AIMING_TIME,
                FieldElements.kSpeakerCenterRed,
                FieldElements.kSpeakerCenterBlue,
                alianceChooser.getSelected().equals(RedAliance)
              )
              .getZ()
        )
      )
    );

    CommandScheduler
      .getInstance()
      .schedule(
        new RepeatCommand(
          new InstantCommand(() ->
            SmartDashboard.putNumber(
              "Drive target",
              (
                Math.toDegrees(
                  Math.atan(
                    -drivebase
                      .getRelativeInterpolatedPosition(
                        driveTimer,
                        Drivebase.AIMING_TIME,
                        FieldElements.kSpeakerCenterRed,
                        FieldElements.kSpeakerCenterBlue
                      )
                      .getY() /
                    -drivebase
                      .getRelativeInterpolatedPosition(
                        driveTimer,
                        Drivebase.AIMING_TIME,
                        FieldElements.kSpeakerCenterRed,
                        FieldElements.kSpeakerCenterBlue
                      )
                      .getX()
                  )
                ) /
                180
              )
            )
          )
        )
      );

    SequentialCommandGroup cancelIntake = new SequentialCommandGroup(
      // Stop intake and stop indexer and tell the robot that the note is not stored
      new InstantCommand(() ->
        CommandScheduler.getInstance().cancel(intakeCommandGroup)
      ),
      new ParallelCommandGroup(
        new IndexerCommand(indexer, true, 0).withTimeout(0.1),
        new IntakeCommand(intake, true, 0).withTimeout(0.1),
        new InstantCommand(() -> noteStatus = OperatorConstants.FALSE)
      ),
      // Stow intake
      new DeployerCommand(intakeDeployer, Intake.HIGH)
    );

    // bind command sequences to buttons

    // if the note is not stored, run intake command sequence. if the intake command sequence is running, cancel the intake command sequence
    new JoystickButton(operatorXbox, 1).onTrue(intakeCommandGroup);

    new JoystickButton(operatorXbox, 3).onTrue(fire);

    new JoystickButton(operatorXbox, 7).onTrue(cancelIntake);

    new JoystickButton(operatorXbox, 4)
      .whileTrue(
        new ParallelCommandGroup(
          new IntakeCommand(intake, true, -0.75),
          new IndexerCommand(indexer, true, 0.2)
        )
      );
    new JoystickButton(operatorXbox, 2)
      .whileTrue(
        new ParallelCommandGroup(
          new IntakeCommand(intake, true, 0.75),
          new IndexerCommand(indexer, true, -0.75)
        )
      );

    new Trigger(() -> operatorXbox.getPOV() == 180)
      .whileTrue(
        new InstantCommand(() -> intakeDeployer.runDeployer(0.3))
          .finallyDo(() ->
            CommandScheduler
              .getInstance()
              .schedule(
                new WaitCommand(0.3)
                  .andThen(
                    new InstantCommand(() -> intakeDeployer.runDeployer(0))
                      .onlyIf(() -> operatorXbox.getPOV() != 180)
                  )
              )
          )
      );
    new Trigger(() -> operatorXbox.getPOV() == 0)
      .whileTrue(
        new InstantCommand(() -> intakeDeployer.runDeployer(-0.5))
          .finallyDo(() ->
            CommandScheduler
              .getInstance()
              .schedule(
                new WaitCommand(0.3)
                  .andThen(
                    new InstantCommand(() -> intakeDeployer.runDeployer(0))
                      .onlyIf(() -> operatorXbox.getPOV() != 0)
                  )
              )
          )
      );
  }







  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SequentialCommandGroup intakeCommandGroup = new SequentialCommandGroup(
      // Start intake and indexer
      new ParallelCommandGroup(
        new ParallelRaceGroup(
          new IntakeCommand(intake, false, -1.0),
          new IndexerCommand(indexer, false, 0.4)
        )
      ),
      // Stop intake and slow indexer
      new ParallelCommandGroup(new IndexerCommand(indexer, false, 0.3)),
      // run indexer backward
      new IndexerCommand(indexer, true, -0.4)
        .withTimeout(0.5)
    );

    //NamedCommands.registerCommand("Stop Shooter", new ShooterCommand(shooter, 0, 0));

    ParallelCommandGroup aimCommandGroup = new ParallelCommandGroup(
      new InstantCommand(pivotTimer::start),
      new PivotCommand(
        pivot,
        () ->
          Math.sqrt(
            Math.pow(
              -drivebase
                .getRelativeInterpolatedPosition(
                  pivotTimer,
                  Shooter.AIMING_TIME,
                  FieldElements.kSpeakerCenterRed,
                  FieldElements.kSpeakerCenterBlue,
                  alianceChooser.getSelected().equals(RedAliance)
                )
                .getX(),
              2
            ) +
            Math.pow(
              -drivebase
                .getRelativeInterpolatedPosition(
                  pivotTimer,
                  Shooter.AIMING_TIME,
                  FieldElements.kSpeakerCenterRed,
                  FieldElements.kSpeakerCenterBlue,
                  alianceChooser.getSelected().equals(RedAliance)
                )
                .getY(),
              2
            )
          ),
        () ->
          -drivebase
            .getRelativeInterpolatedPosition(
              pivotTimer,
              Shooter.AIMING_TIME,
              FieldElements.kSpeakerCenterRed,
              FieldElements.kSpeakerCenterBlue,
              alianceChooser.getSelected().equals(RedAliance)
            )
            .getZ()
      )
        .andThen(new InstantCommand(pivotTimer::reset))
    );

    PivotCommand aimLow = new PivotCommand(pivot, 25);

    NamedCommands.registerCommand(
      "Lower Intake",
      new DeployerCommand(intakeDeployer, Intake.LOW)
    );
    NamedCommands.registerCommand(
      "Raise Intake",
      new DeployerCommand(intakeDeployer, Intake.HIGH)
    );

    NamedCommands.registerCommand("Start Shooter", new InstantCommand(()->shooter.runShooter(-1, false, -0.35)));

    NamedCommands.registerCommand("Stop Shooter", new InstantCommand(()->shooter.runShooter(0, false, 0)));

    NamedCommands.registerCommand(
      "Fire",
      new IndexerCommand(indexer, true, 1).withTimeout(0.75)
    );
    NamedCommands.registerCommand("Intake", intakeCommandGroup);
    NamedCommands.registerCommand("Start Aim", new InstantCommand(()-> aimCommandGroup.schedule()));
    NamedCommands.registerCommand(
      "Stop Aim",
      new ParallelCommandGroup(new InstantCommand(() ->
        CommandScheduler.getInstance().cancel(aimCommandGroup)
      ), new InstantCommand(()-> aimLow.schedule()))
    );

    return drivebase.getAutonomousCommand(autonChooser.getSelected());
  }





  public void setPivotCommand() {
    new RepeatCommand( 
      new InstantCommand(() ->
        SmartDashboard.putNumber(
          "Dist",
          Math.sqrt(
            Math.pow(
              -drivebase
                .getRelativeInterpolatedPosition(
                  pivotTimer,
                  Shooter.AIMING_TIME,
                  FieldElements.kSpeakerCenterRed,
                  FieldElements.kSpeakerCenterBlue,
                  alianceChooser.getSelected().equals(RedAliance)
                )
                .getX(),
              2
            ) +
            Math.pow(
              -drivebase
                .getRelativeInterpolatedPosition(
                  pivotTimer,
                  Shooter.AIMING_TIME,
                  FieldElements.kSpeakerCenterRed,
                  FieldElements.kSpeakerCenterBlue,
                  alianceChooser.getSelected().equals(RedAliance)
                )
                .getY(),
              2
            )
          )
        )
      )
    )
      .schedule();
    new PivotCommand(
          pivot,
          () ->
            SmartDashboard.getNumber("Raw Angle", 9) +
            SmartDashboard.getNumber("Fine Angle", 0) /
            100
        ).schedule();
  }

  public Command startShooter() {
    return new ShooterCommand(shooter, -1, -0.35);
  }

  public void stopShooter() {}

  public boolean isReady() {
    return drivebase.isReady() && pivot.isReady();
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void setShooterOffset() {
    SmartDashboard.putNumber(
      "Calibration",
      pivot.calibrateCameraAngle(LimelightHelpers.getTY(""))
    );
  }
}
