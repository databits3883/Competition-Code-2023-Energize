// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
}
