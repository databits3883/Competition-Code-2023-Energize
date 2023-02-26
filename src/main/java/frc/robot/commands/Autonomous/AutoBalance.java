// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
  final DriveSubsystem m_DriveSubsystem;
  final PIDController m_pitchPidController = new PIDController(0.1, 0.001, 0);
  /** Creates a new AutoBalance. */
  public AutoBalance(DriveSubsystem driveSubsystem) {
    m_DriveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pitchPidController.setTolerance(0.5, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xResult = m_pitchPidController.calculate(m_DriveSubsystem.getGyroRoll(),0);
    m_DriveSubsystem.setSpeedFieldRelative(new ChassisSpeeds( xResult, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pitchPidController.atSetpoint();
  }
}
