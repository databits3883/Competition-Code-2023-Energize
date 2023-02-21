// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTimed extends CommandBase {
  final DriveSubsystem m_DriveSubsystem;
  double driveTimeTotal;
  double driveTimer;
  ChassisSpeeds driveVector;
  /** Creates a new DriveTimed. */
  public DriveTimed(DriveSubsystem driveSubsystem, double time, ChassisSpeeds drive) {
    m_DriveSubsystem = driveSubsystem;
    driveTimeTotal = time;
    driveVector = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTimer = driveTimeTotal;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTimer -= 0.02;
    m_DriveSubsystem.setChassisSpeed(driveVector);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTimer < 0;
  }
}
