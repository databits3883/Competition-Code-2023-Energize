// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.DrivetrainCalibration;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.Run_Cube_Pickup;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

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
  private final PneumaticHub m_PneumaticHub  = new PneumaticHub(GeneralConstants.PNEUMATIC_HUB);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_cubeIntake = new Intake(IntakeConstants.CUBE_LIFTER,IntakeConstants.INTAKE_EXTENDER,IntakeConstants.CONE_LIFTER,IntakeConstants.CONE_EXTENDER,m_PneumaticHub);


  // The driver's controller
  Joystick m_driverStick = new Joystick(0);
  private final Command m_manualDrive = new JoystickDrive(m_robotDrive, m_driverStick);
  private final Command m_calibrateCommand = new DrivetrainCalibration(m_robotDrive);
  private final Command m_turnCommand = new DriveTurnToAngle(m_robotDrive, 1);
  private final Command m_cubeIntakeCommand = new Run_Cube_Pickup(m_cubeIntake);

  private final JoystickButton m_calibrateButton = new JoystickButton(m_driverStick, 8);
  private final JoystickButton m_turnButton = new JoystickButton(m_driverStick, 7);
  private final JoystickButton m_intakeButton = new JoystickButton(m_driverStick, 3);

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

    m_calibrateButton.onTrue(m_calibrateCommand);
    m_turnButton.toggleOnTrue(m_turnCommand);
    m_intakeButton.onTrue(m_cubeIntakeCommand);
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
