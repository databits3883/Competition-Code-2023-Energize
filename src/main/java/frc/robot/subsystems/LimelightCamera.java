// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightCamera extends SubsystemBase {

  public final NetworkTable m_camTable;
  /** Creates a new LimelightCamera. */
  public LimelightCamera() {
    m_camTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void setPipeline(int index){
    m_camTable.getEntry("pipeline").setNumber(index);
  }

  public boolean hasTarget(){
    return m_camTable.getEntry("tv").getDouble(-1) > 0;
  }

  public double getXError(){
    return m_camTable.getEntry("tx").getDouble(-1);
  }

  public double getYError(){
    return m_camTable.getEntry("ty").getDouble(-1);
  }

  public Pose2d getTargetPose(){
    double[] poseList = m_camTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    return new Pose2d(poseList[2], poseList[0], Rotation2d.fromDegrees(poseList[4])); 
  }

  public   Pose2d getSubstationPose(){
    double[] poseList = m_camTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    return new Pose2d(poseList[0], poseList[2], Rotation2d.fromDegrees(poseList[4])); 
  }

  public Pose2d getRobotPoseRedField(){
    double[] poseList = m_camTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    return new Pose2d(poseList[0], poseList[1], Rotation2d.fromDegrees(poseList[5] + 180)); 
  }

  public Pose2d getRobotPoseBlueField(){
    double[] poseList = m_camTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    return new Pose2d(poseList[0], poseList[1], Rotation2d.fromDegrees(poseList[5] + 180)); 
  }

  public Pose2d getRobotColoredFieldPose(){
    boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false);

    if(isRed){
      return getRobotPoseRedField();
    }
    else{
      return getRobotPoseBlueField();
    }
  }
}
