// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ElbowMotorConstants;
import frc.robot.Constants.ArmConstants.ElevatorMotorConstants;
import frc.robot.commands.SetElbowPosition;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ReachPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReachToPositionAuto extends SequentialCommandGroup {
  
  /** Creates a new ReachToPosition. */
  public ReachToPositionAuto(ArmSubsystem arm, ReachPosition reachPos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double elevatorPos = 0;
    double elbowPos = 0;
    switch (reachPos){
      case CUBE_HIGH:
        elevatorPos = ElevatorMotorConstants.CUBE_PLACE_HIGH;
        elbowPos = ElbowMotorConstants.CUBE_PLACE_HIGH;
        break;
      case CUBE_LOW:
        elevatorPos = ElevatorMotorConstants.CUBE_PLACE_LOW;
        elbowPos = ElbowMotorConstants.CUBE_PLACE_LOW;
        break;
      case CUBE_PICKUP:
        elevatorPos = ElevatorMotorConstants.CUBE_PLACE_PICKUP;
        elbowPos = ElbowMotorConstants.CUBE_PLACE_PICKUP;
        break;
      case CONE_HIGH:
        elevatorPos = ElevatorMotorConstants.CONE_PLACE_HIGH;
        elbowPos = ElbowMotorConstants.CONE_PLACE_HIGH;
        break;
      case CONE_LOW:
        elevatorPos = ElevatorMotorConstants.CONE_PLACE_LOW;
        elbowPos = ElbowMotorConstants.CONE_PLACE_LOW;
        break;
      case CONE_PICKUP:
        elevatorPos = ElevatorMotorConstants.CONE_PLACE_PICKUP;
        elbowPos = ElbowMotorConstants.CONE_PLACE_PICKUP;
      default:
        System.out.println("default is running, it probably shouldn't");
        break;
    }
    addCommands(
      new SetElevatorPosition(arm,elevatorPos),
      new SetElbowPosition(arm,elbowPos)
    );
  }
}
