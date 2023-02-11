// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.stream.events.StartDocument;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunCubePickup extends CommandBase {
  /** Creates a new Run_Cube_Pickup. */
  final Intake m_intakeSystem;

  public RunCubePickup(Intake pickupIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSystem = pickupIntake;
    addRequirements(m_intakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSystem.setCubePickupIntake(0.1);
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSystem.setCubePickupIntake(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSystem.setCubePickupIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
