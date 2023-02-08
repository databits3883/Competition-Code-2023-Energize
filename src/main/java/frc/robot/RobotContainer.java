// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Source Control Information
  private final RobotSourceCodeInformation m_RobotSourceCodeInformation = new RobotSourceCodeInformation();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  Joystick m_driverStick = new Joystick(0);
  private final Command m_manualDrive = new JoystickDrive(m_robotDrive, m_driverStick);
  private final Command m_turnCommand = new DriveTurnToAngle(m_robotDrive, 1);
  private final Command m_resetGyro = new ResetGyro(m_robotDrive);

  private final JoystickButton m_resetGyroButton = new JoystickButton(m_driverStick, 8);
  private final JoystickButton m_turnButton = new JoystickButton(m_driverStick, 7);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // load robot source code information
    m_RobotSourceCodeInformation.init();
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand( m_manualDrive);
    //m_calibrateCommand.initialize();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_turnButton.toggleOnTrue(m_turnCommand);
    m_resetGyroButton.onTrue(m_resetGyro);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return null;
  }
}
