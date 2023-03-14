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
  ProfiledPIDController thetaController = new ProfiledPIDController(1.0,0,0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

  public TrajectoryFollowBase(Trajectory trajectory, DriveSubsystem drivetrain, Supplier<Pose2d> poseSupplier) {

    //
    // DCH These PID loops (turning) seem to be very hard to control unless way overdamped
    //

    super(trajectory,
      poseSupplier,
      KINEMATICS, 
      new PIDController(4.0, 0, 0), /*was kp 1 */
      new PIDController(4.0, 0, 0), /*was kp 1 */
      new ProfiledPIDController(0.6, 0.0, 0, /* was Kp 4 */
      // DCH I believe this PID has to be external so that enableContinousInput can be done. So this routine can't be in the constructor.
        new TrapezoidProfile.Constraints(Math.PI, Math.PI /*MAX_TURN_SPEED, MAX_TURN_SPEED*10*/)),
      drivetrain::setStates,
      drivetrain
      );
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      thetaController.reset(null);
      drivetrain.setDisplayTrajectory(trajectory);
  }

}
