// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import javax.sound.sampled.Port;

import constants.Constants.OperatorConstants;
import constants.Ports;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem SwerveDrive  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));

/* Controllers */
private final XboxController DRIVER = new XboxController(Ports.Gamepad.DRIVER);
private final XboxController OPERATOR = new XboxController(Ports.Gamepad.OPERATOR);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(SwerveDrive,
                                                                   () -> MathUtil.applyDeadband(DRIVER.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(DRIVER.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(DRIVER.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   DRIVER::getYButtonPressed,
                                                                   DRIVER::getAButtonPressed,
                                                                   DRIVER::getXButtonPressed,
                                                                   DRIVER::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = SwerveDrive.driveCommand(
        () -> MathUtil.applyDeadband(DRIVER.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(DRIVER.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> DRIVER.getRightX(),
        () -> DRIVER.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = SwerveDrive.driveCommand(
        () -> MathUtil.applyDeadband(DRIVER.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(DRIVER.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> DRIVER.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = SwerveDrive.simDriveCommand(
        () -> MathUtil.applyDeadband(DRIVER.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(DRIVER.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> DRIVER.getRawAxis(2));

    SwerveDrive.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
     // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(DRIVER, 1).onTrue((new InstantCommand(SwerveDrive::zeroGyro)));
    //new JoystickButton(DRIVER, 3).whileTrue(new RepeatCommand(new InstantCommand(SwerveDrive::lock, SwerveDrive)));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return SwerveDrive.getAutonomousCommand("New Path", true);
  }
  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    SwerveDrive.setMotorBrake(brake);
  }
}
