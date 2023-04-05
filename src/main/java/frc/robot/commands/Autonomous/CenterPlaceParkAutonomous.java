// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChangeElbowPosition;
import frc.robot.commands.DrivetrainCalibration;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.ReachToPosition;
import frc.robot.commands.SetArmLiftPosition;
import frc.robot.commands.TheClawGrip;
import frc.robot.commands.Autonomous.DriveTimed;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.ReachPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterPlaceParkAutonomous extends SequentialCommandGroup {
  final DriveSubsystem m_DriveSubsystem;
  final ArmSubsystem m_Arm;
  /** Creates a new CenterPlaceParkAutonomous. */
  public CenterPlaceParkAutonomous(DriveSubsystem driveSubsystem, ArmSubsystem arm) {

    m_DriveSubsystem = driveSubsystem;
      
      m_Arm = arm;

      int ySign = 0;//is undefined

      int xSign = 1;

      // if(isBlue){
      //     ySign = 1;//is blue
      //     System.out.println("Auto in Blue");
      // }
      // else
      // {
      //   ySign = -1;//is red
      //   System.out.println("Auto in Red");
      // }




      //

      //IMPORTANT: DOES NOT TAKE TEAM INTO ACCOUNT, PLACE POSITION IS ALWAYS CONE_HIGH, and THIS IS NOT TESTED

        //
        //read ^





        Command calibrateCommand = new DrivetrainCalibration(m_DriveSubsystem, 180)
        .andThen(new DrivetrainCalibration(m_DriveSubsystem, 180))
        .andThen(new DrivetrainCalibration(m_DriveSubsystem, 180))
        .andThen(new DrivetrainCalibration(m_DriveSubsystem, 180));
        

    Command firstReachCommands =  new ReachToPosition(m_Arm, ReachPosition.CONE_HIGH)
        .andThen(new WaitCommand(0.5))
        .andThen(new DriveTimed(m_DriveSubsystem, 0.2, new ChassisSpeeds(1*xSign, 0*ySign, 0)))
        .andThen(new SetArmLiftPosition(true,m_Arm))
        .andThen(new WaitCommand(1))
        .andThen(new ChangeElbowPosition(m_Arm, 0.05))
        .andThen(new WaitCommand(0.5))
        .andThen(new TheClawGrip(true, m_Arm))
        .andThen(new WaitCommand(0.5));

    Command exitSetup = 
        new DriveTimed(m_DriveSubsystem, 0.4, new ChassisSpeeds(-1*xSign,0*ySign, 0))
        .alongWith(new SetArmLiftPosition(false,m_Arm))
        .alongWith(new WaitCommand(0.25))
        .andThen(new ReachToPosition(m_Arm, ReachPosition.CUBE_PICKUP))
        .alongWith(new WaitCommand(0.5));

    Command exitCommands = 
      new DriveTimed(driveSubsystem, 4.1/1.5, new ChassisSpeeds(-1.5*xSign, 0*ySign, 0));

    Command balanceCommands = 
        new WaitCommand(0.25)
        .andThen(new DriveTimed(m_DriveSubsystem, 2.5, new ChassisSpeeds(0.85*xSign,0*ySign, 0)))
        .andThen(new AutoBalance(m_DriveSubsystem));  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(calibrateCommand,
    firstReachCommands,
    exitSetup,
    exitCommands,
    balanceCommands
    );
  }
}
