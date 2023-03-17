// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DrivetrainCalibration extends CommandBase {
  final DriveSubsystem m_drivetrain;
  final double angleOffset;
  double timeTaken = 0, lastTimeRan = 0;
  /** Creates a new DrivetraintCalibration. */
  public DrivetrainCalibration(DriveSubsystem drivetrain,double offset) {
    angleOffset = offset;
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    
    addRequirements(m_drivetrain);
  }
  @Override
  public void initialize(){
    //m_drivetrain.calibrate();
    //m_drivetrain.resetGyro();
    timeTaken = 0;
    lastTimeRan = System.currentTimeMillis();
    m_drivetrain.calibrate(angleOffset);
    
  }

  @Override
  public void execute() {
    //timeTaken += 0.02;
  }

  @Override
  public void end(boolean interrupted) {
      System.out.print("Finished");
  }

  @Override
  public boolean isFinished() {
    
    System.out.println("calling is finished " + ((System.currentTimeMillis() - lastTimeRan) > 250));
    return ((System.currentTimeMillis() - lastTimeRan) > 250);
  }
}
