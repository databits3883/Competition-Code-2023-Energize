// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDrive extends CommandBase {
  protected final DriveSubsystem m_drivetrain;
  //private ChassisSpeeds lastX;
  private Joystick m_Joystick;

  ChassisSpeeds m_target = new ChassisSpeeds();

  /** Creates a new JoystickDrive. */
  public JoystickDrive(DriveSubsystem drivetrain, Joystick stick) {
    m_drivetrain = drivetrain;
    m_Joystick = stick;
    StickFilter.forwardAxis = ()-> stick.getY();
    StickFilter.sideAxis = ()-> stick.getX();
    StickFilter.twistAxis =()-> -stick.getTwist();
    addRequirements(drivetrain);
    
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double povAngle = m_Joystick.getPOV();
    if(povAngle != -1){
      m_drivetrain.setSpeedFieldRelativePivot(StickFilter.getCurrentCommand(), povAngle + 90);
    }
    else{
      m_drivetrain.setSpeedFieldRelative(StickFilter.getCurrentCommand());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class StickFilter{
    public static DoubleSupplier forwardAxis;
    public static DoubleSupplier sideAxis;
    public static DoubleSupplier twistAxis;

    public static ChassisSpeeds filter(double yAxis, double xAxis,double twist){
      //deadbanding at less than 7 percent usage
      if(Math.abs(yAxis)<0.07){
        yAxis=0;
      }else{
        yAxis = Math.copySign((Math.abs(yAxis)-0.07)/(1-0.07), yAxis);
      }
      if(Math.abs(xAxis)<0.07){
        xAxis=0;
      }else{
        xAxis = Math.copySign((Math.abs(xAxis)-0.07)/(1-0.07), xAxis);
      }

      //cut off corner saturation but preserve direction
      double squareMag = xAxis*xAxis+yAxis*yAxis;
      if(squareMag > 1){
        double mag = Math.sqrt(squareMag);
        yAxis/=mag;
        xAxis/=mag;
      }

      return new ChassisSpeeds(xAxis,yAxis, twist);
    }

    public static ChassisSpeeds getCurrentCommand(){
      ChassisSpeeds s = filter(sideAxis.getAsDouble(), forwardAxis.getAsDouble(), twistAxis.getAsDouble());
      s.vxMetersPerSecond*=Constants.DriveConstants.MAX_WHEEL_SPEED;
      s.vyMetersPerSecond*=Constants.DriveConstants.MAX_WHEEL_SPEED;
      s.omegaRadiansPerSecond*=Constants.DriveConstants.MAX_TURN_SPEED;
      return s;
    }
  }
}
