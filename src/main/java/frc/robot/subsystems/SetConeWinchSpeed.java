// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetConeWinchSpeed extends CommandBase {

  final Intake m_Intake;
  double runSpeed;
  /** Creates a new SetConeWinchSpeed. */
  public SetConeWinchSpeed(Intake daIntakeThingamabob,double vroomRate) {
    m_Intake = daIntakeThingamabob;
    runSpeed = vroomRate;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.setConeWinchSpeed(runSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.setConeWinchSpeed(runSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.setConeWinchSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
