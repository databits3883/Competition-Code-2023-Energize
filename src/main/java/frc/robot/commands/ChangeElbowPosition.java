// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.DelayQueue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChangeElbowPosition extends InstantCommand {
  final ArmSubsystem m_armSubsystem;
  final double change;
  public ChangeElbowPosition(ArmSubsystem armSubsystem, double delta) {
    change = delta;
    m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.changeElbowPosiiton(change);
    System.out.println("Changing setpoint by "+ change);
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

      return m_armSubsystem.atElbowSetpoint();
  }
}
