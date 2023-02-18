// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class RaiseConeWinch extends CommandBase {

  final Intake m_Intake;
  /** Creates a new SetConeWinchSpeed. */
  public RaiseConeWinch(Intake daIntakeThingamabob) {
    m_Intake = daIntakeThingamabob;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.setConeLifterPosition(IntakeConstants.CONE_LIFTER_UP);
  }


@Override
public void end(boolean interrupted) {
    m_Intake.setConeLifterPosition(IntakeConstants.CONE_LIFTER_DOWN);
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
