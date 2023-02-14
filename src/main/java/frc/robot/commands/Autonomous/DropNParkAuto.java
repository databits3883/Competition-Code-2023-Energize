// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.TheClawGrip;
import frc.robot.commands.SetConeSpear;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropNParkAuto extends SequentialCommandGroup {

  final DriveSubsystem m_DriveSubsystem;
  final Intake m_Intake;
  final ArmSubsystem m_Arm;

  final List<Translation2d> toPark_waypoints = List.of(new Translation2d(0,0.5));
  final Trajectory toPark = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0,Rotation2d.fromDegrees(0)), 
    toPark_waypoints, 
    new Pose2d(1,0,Rotation2d.fromDegrees(0)), 
    Constants.DriveConstants.CONFIG);

  final TrajectoryFollowBase parkTrajectory;
  /** Creates a new DropNParkAuto. */
  public DropNParkAuto(DriveSubsystem driveSubsystem, Intake intake, ArmSubsystem arm) {
    m_DriveSubsystem = driveSubsystem;
    m_Intake = intake;
    m_Arm = arm;

    parkTrajectory = new TrajectoryFollowRelative(toPark,m_DriveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TheClawGrip(false, m_Arm),
      parkTrajectory
      
    );
  }
}
