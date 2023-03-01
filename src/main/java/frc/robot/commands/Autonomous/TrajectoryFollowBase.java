// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

public abstract class TrajectoryFollowBase extends SwerveControllerCommand {
  /** Creates a new TrajectoryFollowBase. */
  public TrajectoryFollowBase(Trajectory trajectory, DriveSubsystem drivetrain, Supplier<Pose2d> poseSupplier) {
    super(trajectory,
      poseSupplier,
      KINEMATICS, 
      new PIDController(1, 0, 0), 
      new PIDController(1, 0, 0), 
      new ProfiledPIDController(4, 0, 0, 
        new TrapezoidProfile.Constraints(MAX_TURN_SPEED, MAX_TURN_SPEED*10)),
      drivetrain::setStates,
      drivetrain
      );
      drivetrain.setDisplayTrajectory(trajectory);
  }

}
