// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
  final DriveSubsystem m_DriveSubsystem;
  final PIDController m_rollPidController = new PIDController(0.1, 0, 0);
  double controllerLastOutput;
  double shouldReverse = 0;
  /** Creates a new AutoBalance. */
  public AutoBalance(DriveSubsystem driveSubsystem) {
    m_DriveSubsystem = driveSubsystem;
    m_rollPidController.setTolerance(2,2);
    Shuffleboard.getTab("Tab5").addDouble("Balance Output", () -> controllerLastOutput);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rollPidController.setTolerance(10, 5);
    shouldReverse = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xResult = m_rollPidController.calculate(m_DriveSubsystem.getGyroRoll(),0);
    //xResult = (1-DriveConstants.AUTO_BALANCE_FF)*xResult + (DriveConstants.AUTO_BALANCE_FF) * controllerLastOutput; 

    if(Math.abs(xResult) > 0.3){
      xResult = Math.signum(xResult) * 0.3;
    }
    
    m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(xResult, 0, 0));
    controllerLastOutput = xResult;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //Command counterDrive = new DriveTimed(m_DriveSubsystem, 0.25, new ChassisSpeeds(-controllerLastOutput*5, 0, 0));
    m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(0,0.0001,0));
    //counterDrive.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rollPidController.atSetpoint();
    //return false;
  }
}
