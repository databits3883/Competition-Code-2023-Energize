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
  double startTime;
  ChassisSpeeds driveVector;
  /** Creates a new DriveTimed. */
  public DriveTimed(DriveSubsystem driveSubsystem, double time, ChassisSpeeds drive) {
    m_DriveSubsystem = driveSubsystem;
    driveTimeTotal = time;
    driveVector = new ChassisSpeeds(drive.vxMetersPerSecond / 1.6, drive.vyMetersPerSecond / 1.6, drive.omegaRadiansPerSecond); //1.6 is the random St. Patrick's day fudge factor 3/17/2023
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_DriveSubsystem.setSpeedFieldRelative(driveVector);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(0.000001, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((System.currentTimeMillis() - startTime) > 1000*driveTimeTotal);
  }
}
