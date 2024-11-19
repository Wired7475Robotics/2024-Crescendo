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
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
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

  public static final Joystick leftDriverNunchuck = new Joystick(0);
  public static final Joystick rightDriverNunchuck = new Joystick(1);
  public static final String RedAliance = "red";
  public static final String BlueAliance = "blue";

  public static final String twoNoteMid = "2 Note Mid";
  public static final String twoNoteLeft = "2 Note Left";
  public static final String threeNoteMid = "3 Note Mid";
  public static final String threeNoteLeft = "3 Note Left";
  public static final String driveOut = "Simple Drive";
  public static final String oneNote = "1 Note Mid";
  public static final String oneNoteLeft = "1 Note Left";
  public static final String oneNoteRight = "1 Note Right";
  public static final String threeNoteRight = "3 Note Right";

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
    autonChooser.addOption(oneNoteLeft, oneNoteLeft);
    autonChooser.addOption(oneNoteRight, oneNoteRight);
    autonChooser.addOption(threeNoteRight, threeNoteRight);
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
  }







  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
