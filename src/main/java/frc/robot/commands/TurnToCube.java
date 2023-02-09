// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToCube extends CommandBase {
  final PhotonCamera m_camera;
  final DriveSubsystem m_DriveSubsystem;
  /** Creates a new TurnToCube. */
  public TurnToCube(PhotonCamera camera, DriveSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_camera = camera;
    m_DriveSubsystem = drivetrain;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.setPipelineIndex(VisionConstants.kCubePipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetYah = m_camera.getLatestResult().targets.get(0).getYaw();
    m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(0.25,0,-targetYah));
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
