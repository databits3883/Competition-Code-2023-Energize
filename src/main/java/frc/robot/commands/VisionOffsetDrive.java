// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.Console;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class VisionOffsetDrive extends CommandBase {
  public PhotonCamera m_Camera;
  public DriveSubsystem m_drivetrain;
  final int pipelineIndex;
  Transform3d poseError;
  double driveTimer;
  final double driveSpeed;
  Translation3d goalPos;

  boolean xFinished,yFinished,yawFinished;

  /** Creates a new VisionAim. */
  public VisionOffsetDrive(DriveSubsystem driveSubsystem, PhotonCamera camera, int pipeline,double drivingSpeed,Translation3d poseToGetTo) {
    m_Camera = camera;
    m_drivetrain = driveSubsystem;
    driveSpeed = drivingSpeed;

    pipelineIndex = pipeline;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_Camera.setPipelineIndex(pipelineIndex); 
    PhotonTrackedTarget trackedTarget = m_Camera.getLatestResult().getBestTarget();
    if (trackedTarget == null){
      System.out.println("NO Camera targets seen with current pipeline when trying to run Vision Offset Drive");
      poseError = new Transform3d();
      return;
    }

    poseError = trackedTarget.getBestCameraToTarget();
    driveTimer = 0;
    xFinished = false;
    yFinished = false;
    yawFinished = false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drivetrain.setChassisSpeed(new ChassisSpeeds(-driveSpeed * poseError.getX(),-driveSpeed * poseError.getY(), -driveSpeed * poseError.getZ()));
    driveTimer += 0.02;
    xFinished = driveSpeed*driveSpeed - Math.abs(poseError.getX() - goalPos.getX()) < 0.05;
    yFinished = driveSpeed*driveSpeed - Math.abs(poseError.getY() - goalPos.getY()) < 0.05;
    yawFinished = driveSpeed*driveSpeed - Math.abs(poseError.getZ() - goalPos.getZ()) < 3;

    if(xFinished){
      poseError = (new Transform3d(new Translation3d(0, poseError.getY(), poseError.getZ()), poseError.getRotation()));
    }
    if(yFinished){
      poseError = (new Transform3d(new Translation3d(poseError.getX(), 0, poseError.getZ()), poseError.getRotation()));
    }
    if(yawFinished){
      poseError = (new Transform3d(new Translation3d(poseError.getX(), poseError.getY(), 0), poseError.getRotation()));
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setChassisSpeed(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return xFinished && yFinished && yawFinished;
  }
}
