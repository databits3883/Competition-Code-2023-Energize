// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.trajectory.Trajectory;

import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryFollowRelative extends TrajectoryFollowBase {

  private final DriveSubsystem m_drivetrain;
  /** Creates a new TrajectoryFollowRelative. */
  public TrajectoryFollowRelative(Trajectory trajectory, DriveSubsystem drivetrain) {
    super(
      trajectory,drivetrain,drivetrain::getFieldPose
    );

    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drivetrain.setPoseRelative();
    super.initialize();
  }


  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      System.out.println("I just finished a Trajectory!" + !interrupted);
  }



}
