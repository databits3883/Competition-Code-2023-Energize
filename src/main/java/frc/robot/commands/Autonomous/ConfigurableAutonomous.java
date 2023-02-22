// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.TheClawGrip;
import frc.robot.commands.SetElbowPosition;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.DrivetrainCalibration;
import frc.robot.commands.ReachToPosition;
import frc.robot.commands.SetArmLiftPosition;
import frc.robot.commands.SetConeSpear;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ArmSubsystem.ReachPosition;
import frc.robot.Constants.ArmConstants.ElbowMotorConstants;
import frc.robot.Constants.ArmConstants.ElevatorMotorConstants;

public class ConfigurableAutonomous extends SequentialCommandGroup{
    final DriveSubsystem m_DriveSubsystem;
    final Intake m_Intake;
    final ArmSubsystem m_Arm;
  
    final List<Translation2d> toPark_waypoints = List.of(new Translation2d(0,0.02));
    final Trajectory toPark = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,Rotation2d.fromDegrees(0)), 
      toPark_waypoints, 
      new Pose2d(0,0.04,Rotation2d.fromDegrees(0)), 
      Constants.DriveConstants.CONFIG);
  
    final TrajectoryFollowBase parkTrajectory;
    /** Creates a new DropNParkAuto. */
    public ConfigurableAutonomous(DriveSubsystem driveSubsystem, Intake intake, ArmSubsystem arm, BooleanSupplier shouldPark, IntSupplier firstPlacePos) {
      m_DriveSubsystem = driveSubsystem;
      m_Intake = intake;
      m_Arm = arm;
      double elevatorPos;
      double elbowPos;
      elevatorPos = ElevatorMotorConstants.CUBE_PLACE_LOW;
      elbowPos = ElbowMotorConstants.CUBE_PLACE_LOW;

      parkTrajectory = new TrajectoryFollowRelative(toPark,m_DriveSubsystem);
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
        new ReachToPosition(m_Arm,ArmSubsystem.ReachPosition.CONE_PICKUP),
        new WaitCommand(2),
        new SetArmLiftPosition(true, m_Arm),
        new WaitCommand(2),
        new TheClawGrip(true, m_Arm),
        new DriveTimed(m_DriveSubsystem, 4.1, new ChassisSpeeds(-1, 0, 0)),
        new ReachToPosition(m_Arm, ReachPosition.CONE_PICKUP),
        new DriveTimed(m_DriveSubsystem, 1.75, new ChassisSpeeds(0, -1, 0)),
        new DriveTimed(m_DriveSubsystem, 2.5, new ChassisSpeeds(1, 0, 0)),
        new AutoBalance(m_DriveSubsystem)
        // new SetElevatorPosition(arm,elevatorPos),
        // parkTrajectory
      );
    }
}    

