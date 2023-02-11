// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleCubeIntake extends CommandBase {
  final Intake m_Intake;
  double startSpeed;
  /** Creates a new ToggleCubeIntake. */
  public ToggleCubeIntake(Intake intake) {
    m_Intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(startSpeed == 0){
      startSpeed = 1;
    }
    startSpeed = m_Intake.getCubeExtender();
    if(startSpeed == 0){
      startSpeed = 1;
    }
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.setCubeExtend(0.5*startSpeed*-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.setCubeExtend(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
