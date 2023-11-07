// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
public class TwoPieceAutonomous extends SequentialCommandGroup {
  final DriveSubsystem m_DriveSubsystem;
  final ArmSubsystem m_Arm;
  
  /** Creates a new CenterPlaceParkAutonomous. */
  public TwoPieceAutonomous(DriveSubsystem driveSubsystem, ArmSubsystem arm) {

    m_DriveSubsystem = driveSubsystem;
      
      m_Arm = arm;

      int ySign = 1;//is undefined

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





        Command calibrateCommand = new DrivetrainCalibration(m_DriveSubsystem, 180,false)
        .andThen(new DrivetrainCalibration(m_DriveSubsystem, 180,false))
        .andThen(new DrivetrainCalibration(m_DriveSubsystem, 180,false))
        .andThen(new DrivetrainCalibration(m_DriveSubsystem, 180,false));
        

    Command firstReachCommands =  new ReachToPosition(m_Arm, ReachPosition.CONE_HIGH)
        .alongWith(new WaitCommand(0.5))
        .andThen(new DriveTimed(m_DriveSubsystem, 0.2, new ChassisSpeeds(1*xSign, 0*ySign, 0)))
        .andThen(new SetArmLiftPosition(true,m_Arm))
        .alongWith(new WaitCommand(1))
        .andThen(new ChangeElbowPosition(m_Arm, 0.05))
        .alongWith(new WaitCommand(0.5))
        .andThen(new TheClawGrip(true, m_Arm))
        .andThen(new WaitCommand(0.5));

    Command exitSetup = 
        new DriveTimed(m_DriveSubsystem, 2.8/2, new ChassisSpeeds(-2*xSign,0*ySign, 0))
        //.andThen(new SetArmLiftPosition(false,m_Arm))
        //.andThen(new WaitCommand(0.25))
        .alongWith(new ReachToPosition(m_Arm, ReachPosition.CUBE_PICKUP))
        .alongWith(new WaitCommand(0.5));

    Command turnCommand = 
        new DriveTimed(m_DriveSubsystem, 1, new ChassisSpeeds(0,0,(Math.PI) /(1.4)))
        .andThen(new PrintCommand("Turn Take 1"));

    Command turnBackCommand = 
        new DriveTimed(m_DriveSubsystem, 1, new ChassisSpeeds(0,0,-(Math.PI) /(1.4)))
        .andThen(new PrintCommand("Turn Take 1"));

    Command exitCommands = 
      new DriveTimed(driveSubsystem, 2, new ChassisSpeeds(-1*xSign, 0.1*ySign, 0));


    Command pickupCommand = 
        new TheClawGrip(false, m_Arm).alongWith(
        new WaitCommand(0.5));

    Command returnCommand = new DriveTimed(m_DriveSubsystem, 4, new ChassisSpeeds(1*xSign, 0.01*ySign, 0));
        

    Command secondReachCommands =  new ReachToPosition(m_Arm, ReachPosition.CUBE_HIGH)
      .andThen(new WaitCommand(0.5))
      .andThen(new DriveTimed(m_DriveSubsystem, 0.6, new ChassisSpeeds(1*xSign, 0*ySign, 0)))
      .andThen(new TheClawGrip(true, m_Arm))
      .andThen(new WaitCommand(0.5));

    
    // Add your commands in the addCommands() call, e.g.
        
        addCommands(calibrateCommand,
        firstReachCommands,
        exitSetup,
        turnCommand,
        exitCommands,
        pickupCommand,
        turnBackCommand,
        new ReachToPosition(m_Arm, ReachPosition.CUBE_HIGH),
        returnCommand
        
        
    
        );
    
  }
}