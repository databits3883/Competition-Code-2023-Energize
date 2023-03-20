// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.commands.Autonomous.DriveTimed;
import frc.robot.commands.Autonomous.DriveTimedRelative;
import frc.robot.commands.Autonomous.TrajectoryFollowRelative;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupStationVisionTrajectoryDrive extends InstantCommand {
  public final DriveSubsystem m_DriveSubsystem;
  public final LimelightCamera m_Camera;
  public final JoystickButton m_stopButton;
  private double speed = 1;
  private int xSign = 1;

  public List<Pose2d> drivePoses = new ArrayList<Pose2d>();

  public PickupStationVisionTrajectoryDrive(DriveSubsystem drivetrain,LimelightCamera camera, JoystickButton stopButton,int sign) {
    m_DriveSubsystem = drivetrain;
    m_Camera = camera;
    m_stopButton = stopButton;
    xSign = sign;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_Camera.hasTarget()){
      return;
    }
    Pose2d poseError = m_Camera.getSubstationPose();
    Translation2d relativeError = poseError.getTranslation().rotateBy(poseError.getRotation());
    Command drive = new DriveTimedRelative(m_DriveSubsystem, speed, new ChassisSpeeds(-(relativeError.getY() - 1.4)/speed, xSign * (relativeError.getX() - 0.8)/speed, poseError.getRotation().getRadians()/speed));

    //new UpdateFieldOdometry(m_DriveSubsystem, m_Camera).initialize();
    
    //drivePoses.clear();
  
    //drivePoses.add(0, m_DriveSubsystem.getFieldPose());
    //drivePoses.add(1, GeneralConstants.FEEDER_STATION_COORDS);
    //drivePoses.add(1, m_Camera.getRobotColoredFieldPose().transformBy(new Transform2d(new Translation2d(0, 1),Rotation2d.fromDegrees(0))));

    //Trajectory driveTrajectory = TrajectoryGenerator.generateTrajectory(drivePoses,DriveConstants.SLOW_CONFIG.addConstraint(DriveConstants.CONSTRAINT));
    //Command driveCommand = new TrajectoryFollowRelative(driveTrajectory, m_DriveSubsystem);
    //driveCommand.withInterruptBehavior(InterruptionBehavior.kCancelSelf).schedule();
    drive.until(() -> (!m_stopButton.getAsBoolean())).schedule();
  }
}
