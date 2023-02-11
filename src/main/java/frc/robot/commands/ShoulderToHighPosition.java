// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ShoulderToHighPosition extends InstantCommand {
  final ArmSubsystem m_armSubsystem;

  /** Creates a new PickUpCubeCommand. */
  public ShoulderToHighPosition(ArmSubsystem armSubsystem) {
    m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setShoulderPosition(ArmConstants.ShoulderMotorConstants.PLACE_HIGH);
  }
}
