// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UpdateFieldOdometry extends InstantCommand {
  public final DriveSubsystem m_DriveSubsystem;
  public final LimelightCamera m_Camera;
  public UpdateFieldOdometry(DriveSubsystem drivetrain,LimelightCamera camera) {
    
    m_DriveSubsystem = drivetrain;
    m_Camera = camera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_Camera.hasTarget()){
      m_DriveSubsystem.visionUpdateFieldPose(m_Camera.getRobotColoredFieldPose());
    }

  }
}
