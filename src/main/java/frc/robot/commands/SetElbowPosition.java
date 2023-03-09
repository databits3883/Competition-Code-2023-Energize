// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class SetElbowPosition extends CommandBase {
  final ArmSubsystem m_armSubsystem;
  final double setPos;

  /** Creates a new PickUpCubeCommand. */
  public SetElbowPosition(ArmSubsystem armSubsystem, double pos) {
    m_armSubsystem = armSubsystem;
    setPos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setElbowPosition(setPos);
    System.out.println("setting elbow to "+ setPos + "at pos: "+ m_armSubsystem.getElbowEncoder());
  }

  @Override
  public void execute(){
   m_armSubsystem.runElbowPositionControl(); 
  
  }

  @Override
  public void end(boolean interrupted) {
      System.out.println("at setpoint");
  }

  @Override
  public boolean isFinished() {

      return m_armSubsystem.elbow_atSetpoint;
  }
}
