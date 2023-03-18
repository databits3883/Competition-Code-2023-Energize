// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightCamera;

public class PickupStationSpeedDrive extends CommandBase {
  final DriveSubsystem m_DriveSubsystem;
  public final LimelightCamera m_Camera;
  double startTime = 0,xTime = 0, yTime = 0, yawTime = 0,xSign = 1,ySign = 1,yawSign = 1, driveSpeed,turnSpeed, halfPi = Math.PI/2;
  boolean xDone,yDone,yawDone;
  long runTime = 0;
  public Pose2d initalError;
  /** Creates a new PickupStationSpeedDrive. */
  public PickupStationSpeedDrive(DriveSubsystem driveSubsystem, LimelightCamera camera,double speedMetersPerSecond, double radPerSecond) {
    m_DriveSubsystem = driveSubsystem;
    m_Camera = camera;
    driveSpeed = speedMetersPerSecond;
    turnSpeed = radPerSecond;
    


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    initalError = m_Camera.getTargetPose();

    System.out.println("Inital: " + initalError);
    initalError = initalError.transformBy(new Transform2d(new Translation2d(-0.8, -1.4),Rotation2d.fromDegrees(0)));
    System.out.println("Transformed: " + initalError);

    xTime = Math.abs((initalError.getY()  * Math.sqrt(2)));
    yTime = Math.abs((Math.abs(initalError.getX()) - Math.abs(initalError.getY()) ) + xTime);
    yawTime = Math.abs(initalError.getRotation().getRadians()/turnSpeed);

    System.out.println("X Time Predicted " + xTime + "Y Time Predicted " + yTime + "Yaw Time Predicted " + yawTime);

    xTime = xTime / driveSpeed;
    yTime = yTime / driveSpeed;

    System.out.println("After Speed: X Time Predicted " + xTime + "Y Time Predicted " + yTime + "Yaw Time Predicted " + yawTime);

    //xTime = xTime / halfPi;
    //yTime = yTime / halfPi;
    //System.out.println("After halfPI: X Time Predicted " + xTime + "Y Time Predicted " + yTime + "Yaw Time Predicted " + yawTime);

    xSign = Math.signum(initalError.getY());
    ySign = -1 * Math.signum(initalError.getX());
    yawSign = Math.signum(initalError.getRotation().getRadians());

    xDone = false;
    yDone = false;
    yawDone = false;

    runTime = (long) startTime;

    if(!m_Camera.hasTarget()){
      xDone = true;
      yDone = true;
      yawDone = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    double xSpeed = 0,ySpeed = 0,yawSpeed = 0;
    if(!xDone){
      xSpeed = driveSpeed * xSign;
    }

    if(!yDone){
      ySpeed = driveSpeed * ySign;
    }

    if(!yawDone){
      yawSpeed = turnSpeed * yawSign;
    }
    
    m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(ySpeed,xSpeed,yawSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setChassisSpeed(new ChassisSpeeds(0,0,0));
    System.out.println("end time: " + (System.currentTimeMillis() - startTime));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(xDone && yDone && yawDone){
      return true;
    }

    long delta = (long) (System.currentTimeMillis() - startTime);
    xDone = (delta > xTime*1000);
    yDone = (delta > yTime*1000);
    yawDone =(delta > yawTime*1000);

    if((System.currentTimeMillis() - runTime) > 500)
    {
      System.out.println("xDone " + xDone + " yDone " + yDone + " yawDone " + yawDone + " Delta: " + delta);
      runTime = System.currentTimeMillis();
    }


    return xDone && yDone && yawDone;
  }
}
