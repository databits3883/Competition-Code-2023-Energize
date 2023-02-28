// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DrivetrainCalibration extends InstantCommand {
  final DriveSubsystem m_drivetrain;
  final double angleOffset;
  /** Creates a new DrivetraintCalibration. */
  public DrivetrainCalibration(DriveSubsystem drivetrain,double offset) {
    angleOffset = offset;
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
  }
  @Override
  public void initialize(){
    //m_drivetrain.calibrate();
    //m_drivetrain.resetGyro();
    m_drivetrain.calibrate(angleOffset);
  }

  
}
