// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.management.loading.MLetMBean;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTarget extends CommandBase {
  final PhotonCamera m_camera;
  final DriveSubsystem m_DriveSubsystem;
  final PIDController m_aimController = new PIDController(0.03, 0, 0);
  final PIDController m_distanceController = new PIDController(0.05, 0, 0);
  final int m_pipelineIndex = 0;
  final double targetDistFromTarget = 11;
  /** Creates a new TurnToCube. */
  public TurnToTarget(PhotonCamera camera, DriveSubsystem drivetrain, int pipelineIndex) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_camera = camera;    
    m_DriveSubsystem = drivetrain;
    

    addRequirements(m_DriveSubsystem);
    //Shuffleboard.getTab("Tab5").addDouble("last Camera Yaw", () -> m_camera.getLatestResult().getBestTarget().getYaw());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.setPipelineIndex(m_pipelineIndex);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    PhotonTrackedTarget target = m_camera.getLatestResult().getBestTarget();
    
    
    
    if(target!= null){
      double targetYah = -target.getYaw();
      
      double targetPitch = target.getPitch();
      
      m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(m_distanceController.calculate(targetPitch, -targetDistFromTarget),0,m_aimController.calculate(targetYah,0)));
    }
    else {
      //m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(0.1,0.1,0.1));
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
