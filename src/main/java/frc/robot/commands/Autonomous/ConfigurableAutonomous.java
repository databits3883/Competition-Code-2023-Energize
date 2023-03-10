// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.TheClawGrip;
import frc.robot.commands.ChangeElbowPosition;
import frc.robot.commands.DrivetrainCalibration;
import frc.robot.commands.ReachToPosition;
import frc.robot.commands.SetArmLiftPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.ReachPosition;

public class ConfigurableAutonomous extends CommandBase{

    public static boolean shouldPark = false;
    public static boolean shouldExit = false;
    public static boolean shouldPlace = false;
    public static boolean imBlue = true;
    public static ReachPosition placePosition = null;

    final DriveSubsystem m_DriveSubsystem;
    final ArmSubsystem m_Arm;
  
    final List<Translation2d> toPark_waypoints = List.of(new Translation2d(0,0.02));
    final Trajectory toPark = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,Rotation2d.fromDegrees(0)), 
      toPark_waypoints, 
      new Pose2d(0,0.04,Rotation2d.fromDegrees(0)), 
      Constants.DriveConstants.CONFIG);
  
    final TrajectoryFollowBase parkTrajectory;
    /** Creates a new DropNParkAuto. */
    public ConfigurableAutonomous(DriveSubsystem driveSubsystem, ArmSubsystem arm) {
      m_DriveSubsystem = driveSubsystem;
      
      m_Arm = arm;
      

      parkTrajectory = new TrajectoryFollowRelative(toPark,m_DriveSubsystem);
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      
      //  new ReachToPosition(m_Arm, ReachPosition.CONE_HIGH).unless(firstPlace_isCone),
      //  new ReachToPosition(m_Arm, ReachPosition.CONE_LOW),
      //  new ReachToPosition(m_Arm, ReachPosition.CONE_PICKUP),

      //  new ReachToPosition(m_Arm, ReachPosition.CUBE_HIGH),
      //  new ReachToPosition(m_Arm, ReachPosition.CUBE_LOW),
      //  new ReachToPosition(m_Arm, ReachPosition.CUBE_PICKUP),
        
      //  new WaitCommand(2),
      //  new SetArmLiftPosition(true, m_Arm),
      ///  new WaitCommand(2),
      // new TheClawGrip(true, m_Arm),
       // new DriveTimed(m_DriveSubsystem, 4.1, new ChassisSpeeds(-1, 0, 0)),
      //  new ReachToPosition(m_Arm, ReachPosition.CONE_PICKUP),
      //  new DriveTimed(m_DriveSubsystem, 1.75, new ChassisSpeeds(0, -1, 0)),
      //  new DriveTimed(m_DriveSubsystem, 2.5, new ChassisSpeeds(1, 0, 0)),
      //  new AutoBalance(m_DriveSubsystem)
        // new SetElevatorPosition(arm,elevatorPos),
        // parkTrajectory
      
    }

    @Override
    public void initialize(){
      setupCommands(shouldPlace, placePosition, shouldExit, shouldPark, imBlue);
    }

    public void setupCommands(boolean shouldReachFirst,ReachPosition firstReach,boolean shouldExit,boolean shouldPark, boolean isBlue){

      int ySign = 1;//is red

      int xSign = 1;

      if(isBlue){
          ySign = -1;//is blue
          System.out.println("Auto in Blue");
      }

      SequentialCommandGroup finalResultCommand;
      List<Command> resultCommands = new ArrayList<>();

      Command calibrateCommand = new DrivetrainCalibration(m_DriveSubsystem, 180).andThen(new PrintCommand("Calibrating"));
      resultCommands.add(calibrateCommand);

      if(shouldPark){
        shouldExit = true;
      }

      if (shouldReachFirst){
        Command firstReachCommands =  new ReachToPosition(m_Arm, firstReach)
        .andThen(new DriveTimed(m_DriveSubsystem, 0.2, new ChassisSpeeds(1*xSign, 0*ySign, 0)))
        .andThen(new WaitCommand(0.5))
        .andThen(new SetArmLiftPosition(true,m_Arm))
        .andThen(new WaitCommand(1))
        .andThen(new ChangeElbowPosition(m_Arm, 0.05))
        .andThen(new TheClawGrip(true, m_Arm))
        .andThen(new WaitCommand(1));


        resultCommands.add(firstReachCommands);
      }

      if (shouldExit){
        Command exitCommands = new DriveTimed(m_DriveSubsystem, 4.1/2, new ChassisSpeeds(-2*xSign, 0*ySign, 0))
        .andThen(new WaitCommand(0.25))
        .andThen((new ReachToPosition(m_Arm, ReachPosition.CONE_PICKUP))
        .alongWith(new DriveTimed(m_DriveSubsystem, 2.25/1, new ChassisSpeeds(0*xSign, -1, 0))))
        .andThen(new WaitCommand(0.25));
        
        resultCommands.add(exitCommands);
      }


      if(shouldPark){
        Command balanceCommands = new DriveTimed(m_DriveSubsystem, 1, new ChassisSpeeds(2.5*xSign,0*ySign, 0))
        .andThen(new AutoBalance(m_DriveSubsystem));  

        resultCommands.add(balanceCommands);
      }
      
      
      /*for (int i = 0; i < resultCommands.size(); i++){
        
        finalResultCommand = new PrintCommand("Next Step: " + i).andThen(resultCommands.get(i));
      }*/

      
      

      int resultCommandsSize = resultCommands.size();
      switch (resultCommandsSize) {
        case 1:
          finalResultCommand = new SequentialCommandGroup(resultCommands.get(0));
          break;
        case 2:
          finalResultCommand = new SequentialCommandGroup(resultCommands.get(0)).andThen(resultCommands.get(1));
          break;
        case 3:
          finalResultCommand = new SequentialCommandGroup(resultCommands.get(0)).andThen(resultCommands.get(1))
            .andThen(resultCommands.get(2));
          break;
        case 4:
          finalResultCommand = new SequentialCommandGroup(resultCommands.get(0)).andThen(resultCommands.get(1))
            .andThen(resultCommands.get(2))
            .andThen(resultCommands.get(3));
          break;
        default:
          finalResultCommand = new PrintCommand("Not doing anything in Chris Braun").andThen(new PrintCommand("nothing either"));
          break;
      }

      finalResultCommand.schedule();
    }
}    

