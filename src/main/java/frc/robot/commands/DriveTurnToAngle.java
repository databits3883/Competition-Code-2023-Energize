// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveTurnToAngle extends CommandBase {
  final DriveSubsystem m_DriveSubsystem;
  final double angleSpeed;
  public DriveTurnToAngle(DriveSubsystem DRIVETRAIN, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    angleSpeed = angle;
    m_DriveSubsystem = DRIVETRAIN;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
      m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(-0.1,0,0));
  }
}
