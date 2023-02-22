// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DrivetrainCalibration;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.ReachToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.RunCubePickup;
import frc.robot.commands.SetArmLiftPosition;
import frc.robot.commands.TheClawGrip;
import frc.robot.commands.VisionAim;
import frc.robot.commands.SetConeSpear;
import frc.robot.commands.RaiseConeWinch;
import frc.robot.commands.SetCubeIntake;
import frc.robot.commands.Autonomous.AutoBalance;
import frc.robot.commands.Autonomous.ConfigurableAutonomous;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.ArmLift;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ArmSubsystem.ReachPosition;

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
  private final PhotonCamera m_Camera = new PhotonCamera(NetworkTableInstance.getDefault(), "HD_Webcam_C525");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_robotArm = new ArmSubsystem(m_PneumaticHub);
  private final Intake m_intake = new Intake(IntakeConstants.CANChannels.CUBE_PICKUP,IntakeConstants.CANChannels.CUBE_EXTENDER,IntakeConstants.CANChannels.CONE_WINCH,IntakeConstants.PneumaticHubChannels.CONE_SPIKE,m_PneumaticHub);



  private final Command autoBalanceCommand = new AutoBalance(m_robotDrive);


  // The driver's controller
  Joystick m_driverStick = new Joystick(0);
  Joystick m_copilotController = new Joystick(1);


  private final Command m_manualDrive = new JoystickDrive(m_robotDrive, m_driverStick);
  private final Command m_coneAimDrive = new VisionAim(m_robotDrive,m_Camera, m_driverStick,3);
  private final Command m_cubeAimDrive = new VisionAim(m_robotDrive,m_Camera, m_driverStick,1);
  private final Command m_postAimDrive = new VisionAim(m_robotDrive,m_Camera, m_driverStick,4);
  private final Command m_calibrateCommand = new DrivetrainCalibration(m_robotDrive, 180);
  private final Command m_calibrateReversedCommand = new DrivetrainCalibration(m_robotDrive, 0);

  private final Command m_cubePickupCommand = new RunCubePickup(m_intake,1);
  private final Command m_cubeDropCommand = new RunCubePickup(m_intake,-1);
  private final Command m_extendCubeIntakeCommand = new SetCubeIntake(m_intake,1);
  private final Command m_retractCubeIntakeCommand = new SetCubeIntake(m_intake,-1);
  private final Command m_extendConeIntakeCommand = new SetConeSpear(m_intake,true);
  private final Command m_retractConeIntakeCommand = new SetConeSpear(m_intake,false);
  private final Command m_setArmDownCommand = new SetArmLiftPosition(ArmLift.DOWN, m_robotArm);
  private final Command m_setArmUpCommand = new SetArmLiftPosition(ArmLift.UP, m_robotArm);
  private final Command m_openClawCommand = new TheClawGrip(false, m_robotArm);
  private final Command m_closeClawCommand = new TheClawGrip(true, m_robotArm);
  private final Command m_raiseConeWinchCommand = new RaiseConeWinch(m_intake);


  private final Command m_reachCubeHighCommand = new ReachToPosition(m_robotArm, ReachPosition.CUBE_HIGH);
  private final Command m_reachCubeLowCommand = new ReachToPosition(m_robotArm, ReachPosition.CUBE_LOW);
  private final Command m_reachCubePickupCommand = new ReachToPosition(m_robotArm, ReachPosition.CUBE_PICKUP);
  
  private final Command m_reachConeHighCommand = new ReachToPosition(m_robotArm, ReachPosition.CONE_HIGH);
  private final Command m_reachConeLowCommand = new ReachToPosition(m_robotArm, ReachPosition.CONE_LOW);
  private final Command m_reachConePickupCommand = new ReachToPosition(m_robotArm, ReachPosition.CONE_PICKUP);

  private final JoystickButton m_calibrateButton = new JoystickButton(m_driverStick, 7);
  private final JoystickButton m_calibratReversedButton = new JoystickButton(m_driverStick, 8);
  private final JoystickButton m_coneAimDriveButton = new JoystickButton(m_driverStick, 2);
  private final JoystickButton m_cubeAimDriveButton = new JoystickButton(m_driverStick, 4);
  private final JoystickButton m_postAimDriveButton = new JoystickButton(m_driverStick, 3);
  
  private final JoystickButton m_toggleClawButton = new JoystickButton(m_copilotController, 7);
  private final JoystickButton m_extendIntakeButton = new JoystickButton(m_copilotController, 8);
  private final JoystickButton m_retractIntakeButton = new JoystickButton(m_copilotController, 9);
  private final JoystickButton m_cubePickupButton = new JoystickButton(m_copilotController, 5);
  private final JoystickButton m_cubeDropButton = new JoystickButton(m_copilotController, 6);
  private final JoystickButton m_setArmRaiserSwitch = new JoystickButton(m_copilotController, 11);

  private final JoystickButton m_cubeConeSelectorSwitch = new JoystickButton(m_copilotController, 12);
  private final JoystickButton m_coneWinchButton = new JoystickButton(m_copilotController, 4);//no cone winch buttons on button box
  private final JoystickButton m_reachHighButton = new JoystickButton(m_copilotController, 1);
  private final JoystickButton m_reachLowButton = new JoystickButton(m_copilotController, 2);
  private final JoystickButton m_reachPickupButton = new JoystickButton(m_copilotController, 3);

  private final JoystickButton m_autoBalanceButton = new JoystickButton(m_copilotController, 10);

  private final SendableChooser<Boolean> autoShouldPark = new SendableChooser<>();
  private final SendableChooser<Integer> autoFirstPlace = new SendableChooser<>();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  //autonomous commands
  private final Command configurableAutonomous = new ConfigurableAutonomous(m_robotDrive, m_intake,m_robotArm, ()-> autoShouldPark.getSelected(),()-> autoFirstPlace.getSelected());


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // load robot source code information
    m_RobotSourceCodeInformation.init();
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand( m_manualDrive);
    m_PneumaticHub.enableCompressorAnalog(100, 120);

    autoChooser.setDefaultOption("Do Nothing", new PrintCommand("No Autonomous"));
    autoChooser.addOption("Chris Braun", configurableAutonomous);
    

    autoShouldPark.setDefaultOption("True", true);
    autoShouldPark.addOption("False", false);

    //autoFirstPlace.setDefaultOption("NONE", false);
    

    


    Shuffleboard.getTab("Game Screen").add("Autonomous Routine",autoChooser).withSize(1, 1).withWidget(BuiltInWidgets.kToggleSwitch);
    Shuffleboard.getTab("Game Screen").add("Park in Auto?", autoShouldPark).withSize(2, 1);

    Shuffleboard.getTab("Tab5").addDouble("PneumaticHub Pressure", ()->m_PneumaticHub.getPressure(GeneralConstants.PNEUMATIC_HUB_PRESSURE_SENSOR_ID));
    
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
    m_calibratReversedButton.onTrue(m_calibrateReversedCommand);
    m_coneAimDriveButton.whileTrue(m_coneAimDrive);
    m_cubeAimDriveButton.whileTrue(m_cubeAimDrive);
    m_postAimDriveButton.whileTrue(m_postAimDrive);

    m_cubePickupButton.whileTrue(m_cubePickupCommand);
    m_cubeDropButton.whileTrue(m_cubeDropCommand);

    m_setArmRaiserSwitch.onFalse(m_setArmDownCommand);
    m_setArmRaiserSwitch.onTrue(m_setArmUpCommand);

    m_toggleClawButton.onFalse(m_openClawCommand);
    m_toggleClawButton.onTrue(m_closeClawCommand);

    m_coneWinchButton.toggleOnTrue(m_raiseConeWinchCommand);
    //m_coneWinchButton.toggleOnTrue(m_lowerConeWinchCommand);

    m_reachHighButton.and(m_cubeConeSelectorSwitch).onTrue(m_reachConeHighCommand);//cone high
    m_reachLowButton.and(m_cubeConeSelectorSwitch).onTrue(m_reachConeLowCommand);//cone low
    m_reachPickupButton.and(m_cubeConeSelectorSwitch).onTrue(m_reachConePickupCommand);//cone pickup

    m_reachHighButton.and(m_cubeConeSelectorSwitch.negate()).onTrue(m_reachCubeHighCommand);//cube high
    m_reachLowButton.and(m_cubeConeSelectorSwitch.negate()).onTrue(m_reachCubeLowCommand);//cube low
    m_reachPickupButton.and(m_cubeConeSelectorSwitch.negate()).onTrue(m_reachCubePickupCommand);//cube pickup

    m_cubeConeSelectorSwitch.onFalse(m_retractConeIntakeCommand);
    m_cubeConeSelectorSwitch.onTrue(m_retractCubeIntakeCommand);

    m_extendIntakeButton.and(m_cubeConeSelectorSwitch.negate()).onTrue(m_extendCubeIntakeCommand);//cube extend
    m_retractIntakeButton.and(m_cubeConeSelectorSwitch.negate()).onTrue(m_retractCubeIntakeCommand);//cube retract

    m_extendIntakeButton.and(m_cubeConeSelectorSwitch).onTrue(m_extendConeIntakeCommand);//cone extend
    m_retractIntakeButton.and(m_cubeConeSelectorSwitch).onTrue(m_retractConeIntakeCommand);//cone retract
    
    m_autoBalanceButton.whileTrue(autoBalanceCommand);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("autonomous" + autoChooser.getSelected().toString());
    
    return autoChooser.getSelected();
  }
}
