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
  public PIDController aimController = new PIDController(0.01, 0.0001, 0);

  /** Creates a new VisionAim. */
  public VisionAim(DriveSubsystem driveSubsystem, PhotonCamera camera, Joystick stick) {
    m_Camera = camera;
    m_drivetrain = driveSubsystem;
    m_stick = stick;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double povAngle = m_stick.getPOV();
    ChassisSpeeds speeds = JoystickDrive.StickFilter.getCurrentCommand();

    double aimOutput = aimController.calculate(m_Camera.getLatestResult().getBestTarget().getYaw());

    speeds.omegaRadiansPerSecond += aimOutput;
    if(povAngle != -1){
      m_drivetrain.setSpeedFieldRelativePivot(speeds, povAngle + 90);
    }
    else{
      m_drivetrain.setSpeedFieldRelative(speeds);
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
