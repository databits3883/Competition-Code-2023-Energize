// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.stream.events.StartDocument;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Run_Cube_Pickup extends CommandBase {
  /** Creates a new Run_Cube_Pickup. */
  final Intake cubeIntakeSystem;

  public Run_Cube_Pickup(Intake pickupIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    cubeIntakeSystem = pickupIntake;
    addRequirements(cubeIntakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cubeIntakeSystem.setCubeIntake(.5);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cubeIntakeSystem.setCubeIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
