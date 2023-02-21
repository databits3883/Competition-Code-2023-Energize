// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class VisionAim extends CommandBase {
  public PhotonCamera m_Camera;
  public DriveSubsystem m_drivetrain;
  public Joystick m_stick;
  public PIDController aimController = new PIDController(0.075, 0.001, 0);
  final int pipelineIndex;

  /** Creates a new VisionAim. */
  public VisionAim(DriveSubsystem driveSubsystem, PhotonCamera camera, Joystick stick, int pipeline) {
    m_Camera = camera;
    m_drivetrain = driveSubsystem;
    m_stick = stick;

    pipelineIndex = pipeline;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_Camera.setPipelineIndex(pipelineIndex); 
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Camera.getLatestResult().getBestTarget() == null){
      return;
    }
    double aimOutput = aimController.calculate(m_Camera.getLatestResult().getBestTarget().getYaw());

    m_drivetrain.setChassisSpeed(new ChassisSpeeds(-m_stick.getY(),-m_stick.getX(), aimOutput));
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
