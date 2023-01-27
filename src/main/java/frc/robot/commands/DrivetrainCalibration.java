// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DrivetrainCalibration extends CommandBase {
  final DriveSubsystem m_drivetrain;
  /** Creates a new DrivetraintCalibration. */
  public DrivetrainCalibration(DriveSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
  }
  @Override
  public void initialize(){
    if(!m_drivetrain.getAllCalibrated()){
      m_drivetrain.calibrate();
    }else{
      m_drivetrain.resetGyro();
    }

  }

  @Override
  public void execute(){
    return;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.getAllCalibrated();
  }
}
