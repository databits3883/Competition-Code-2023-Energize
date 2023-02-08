// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  final CANSparkMax cubeIntakeMotor;
  final RelativeEncoder cubeEncoder; 
  
  public Intake(int cubeIntakeMotorChannel) {
    cubeIntakeMotor = new CANSparkMax(cubeIntakeMotorChannel, MotorType.kBrushless); 
    cubeEncoder = cubeIntakeMotor.getEncoder();
  }

  public void setCubeIntake(double cubeSpeed) {
    // This method will be called once per scheduler run
    cubeIntakeMotor.set(cubeSpeed); 
    
  }

  public double getCubeEncoder() {
    double cubeRotations = cubeEncoder.getVelocity();
    return cubeRotations;
  }
}
