// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.DrivetrainCalibration;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.RunCubePickup;
import frc.robot.commands.SetArmLiftPosition;
import frc.robot.commands.ElbowToHighPosition;
import frc.robot.commands.ElbowToPickupPosition;
import frc.robot.commands.TheClawGrip;
import frc.robot.commands.SetConeSpear;
import frc.robot.commands.SetCubeIntake;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.Autonomous.DropNParkAuto;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.ArmLift;
import frc.robot.Constants.ArmConstants.ElbowMotorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SetConeWinchSpeed;

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
  private final PneumaticHub m_PneumaticHub  = new PneumaticHub(GeneralConstants.PNEUMATIC_HUB_CAN_CHANNEL);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_robotArm = new ArmSubsystem(m_PneumaticHub);
  private final Intake m_intake = new Intake(IntakeConstants.CANChannels.CUBE_PICKUP,IntakeConstants.CANChannels.CUBE_EXTENDER,IntakeConstants.CANChannels.CONE_WINCH,IntakeConstants.PneumaticHubChannels.CONE_SPIKE,m_PneumaticHub);

  //autonomous commands
  private final Command dropNParkAuton = new DropNParkAuto(m_robotDrive, m_intake,m_robotArm);


  // The driver's controller
  Joystick m_driverStick = new Joystick(0);
  Joystick m_copilotController = new Joystick(1);


  private final Command m_manualDrive = new JoystickDrive(m_robotDrive, m_driverStick);
  private final Command m_calibrateCommand = new DrivetrainCalibration(m_robotDrive);
  private final Command m_turnCommand = new DriveTurnToAngle(m_robotDrive, 1);
  private final Command m_cubePickupCommand = new RunCubePickup(m_intake,1);
  private final Command m_cubeDropCommand = new RunCubePickup(m_intake,-1);
  private final Command m_extendCubeIntakeCommand = new SetCubeIntake(m_intake,1);
  private final Command m_retractCubeIntakeCommand = new SetCubeIntake(m_intake,-1);
  private final Command m_extendConeIntakeCommand = new SetConeSpear(m_intake,true);
  private final Command m_retractConeIntakeCommand = new SetConeSpear(m_intake,false);
  private final Command m_setArmDownCommand = new SetArmLiftPosition(ArmLift.DOWN, m_robotArm);
  private final Command m_setArmUpCommand = new SetArmLiftPosition(ArmLift.UP, m_robotArm);
  private final Command m_setElbowHigh = new ElbowToHighPosition(m_robotArm);
  private final Command m_setElbowPickup = new ElbowToPickupPosition(m_robotArm);
  private final Command m_openClawCommand = new TheClawGrip(false, m_robotArm);
  private final Command m_closeClawCommand = new TheClawGrip(true, m_robotArm);
  private final Command m_raiseConeWinchCommand = new SetConeWinchSpeed(m_intake, 1);
  private final Command m_lowerConeWinchCommand = new SetConeWinchSpeed(m_intake, -1);
  private final Command m_setElevatorHigh = new SetElevatorPosition(m_robotArm, ElbowMotorConstants.PLACE_HIGH);
  private final Command m_setElevatorLow = new SetElevatorPosition(m_robotArm, ElbowMotorConstants.PLACE_LOW);
  private final Command m_setElevatorPickup = new SetElevatorPosition(m_robotArm, ElbowMotorConstants.PLACE_PICKUP);

  private final JoystickButton m_calibrateButton = new JoystickButton(m_driverStick, 8);
  private final JoystickButton m_turnButton = new JoystickButton(m_driverStick, 7);
  
  private final JoystickButton m_toggleClawButton = new JoystickButton(m_copilotController, 1);
  private final JoystickButton m_extendCubeIntakeButton = new JoystickButton(m_copilotController, 12);
  private final JoystickButton m_retractCubeIntakeButton = new JoystickButton(m_copilotController, 15);
  private final JoystickButton m_cubePickupButton = new JoystickButton(m_copilotController, 11);
  private final JoystickButton m_cubeDropButton = new JoystickButton(m_copilotController, 16);
  private final JoystickButton m_extendConeIntakeSpearButton = new JoystickButton(m_copilotController, 3);
  private final JoystickButton m_retractConeIntakeSpearButton = new JoystickButton(m_copilotController, 4);
  private final JoystickButton m_setArmDownButton = new JoystickButton(m_copilotController, 8);
  private final JoystickButton m_setArmUpButton = new JoystickButton(m_copilotController, 7);
  private final JoystickButton m_raiseConeWinchButton = new JoystickButton(m_copilotController, 13);
  private final JoystickButton m_lowerConeWinchButton = new JoystickButton(m_copilotController, 14);
  private final JoystickButton m_setElbowPickupButton = new JoystickButton(m_copilotController, 6);
  private final JoystickButton m_setElbowHighButton = new JoystickButton(m_copilotController, 9);
  private final JoystickButton m_setElevatorHighButton = new JoystickButton(m_copilotController, 5);
  private final JoystickButton m_setElevatorLowButton = new JoystickButton(m_copilotController, 10);
  private final JoystickButton m_setElevatorPickupButton = new JoystickButton(m_copilotController, 10);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // load robot source code information
    m_RobotSourceCodeInformation.init();
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand( m_manualDrive);
    m_PneumaticHub.enableCompressorAnalog(5, 120);
    Shuffleboard.getTab("Tab5").addDouble("PneumaticHub", ()->m_PneumaticHub.getPressure(GeneralConstants.PNEUMATIC_HUB_PRESSURE_SENSOR_ID));
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

    m_cubePickupButton.onTrue(m_cubePickupCommand);
    m_cubeDropButton.onTrue(m_cubeDropCommand);
    m_extendConeIntakeSpearButton.onTrue(m_extendConeIntakeCommand);
    m_retractConeIntakeSpearButton.onTrue(m_retractConeIntakeCommand);
    m_extendCubeIntakeButton.onTrue(m_extendCubeIntakeCommand);
    m_retractCubeIntakeButton.onTrue(m_retractCubeIntakeCommand);
    m_setArmDownButton.onTrue(m_setArmDownCommand);
    m_setArmUpButton.onTrue(m_setArmUpCommand);
    m_toggleClawButton.onFalse(m_openClawCommand);
    m_toggleClawButton.onTrue(m_closeClawCommand);
    m_raiseConeWinchButton.onTrue(m_raiseConeWinchCommand);
    m_lowerConeWinchButton.onTrue(m_lowerConeWinchCommand);
    m_setElbowPickupButton.onTrue(m_setElbowPickup);
    m_setElbowHighButton.onTrue(m_setElbowHigh);
    m_setElevatorHighButton.onTrue(m_setElevatorHigh);
    m_setElevatorLowButton.onTrue(m_setElevatorLow);
    m_setElevatorPickupButton.onTrue(m_setElevatorPickup);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return dropNParkAuton;
  }
}
