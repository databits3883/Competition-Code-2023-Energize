// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.CacheRequest;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  final CANSparkMax m_cubePickupMotor;
  final RelativeEncoder m_cubePickupEncoder;
  final CANSparkMax m_cubeDeployMotor;
  double lastCubeDeploySpeed = 0;
  final CANSparkMax m_conePickupLiftMotor;
  final PneumaticHub m_pneumaticHub;
  final Solenoid m_coneExtendSolenoid;
  final SparkMaxPIDController m_coneLifterPID;
  final RelativeEncoder m_coneLifterEncoder;

  
  public Intake(int cubeIntakeMotorChannel, int cubeIntakeDeployMotorChannel, int coneLifterChannel, int coneExtendChannel, PneumaticHub PneumaticHub) {
    m_pneumaticHub = PneumaticHub;
    m_cubePickupMotor = new CANSparkMax(cubeIntakeMotorChannel, MotorType.kBrushless);
    m_cubeDeployMotor = new CANSparkMax(cubeIntakeDeployMotorChannel, MotorType.kBrushed);
    m_conePickupLiftMotor = new CANSparkMax(coneLifterChannel, MotorType.kBrushless);
    m_coneExtendSolenoid = m_pneumaticHub.makeSolenoid(coneExtendChannel);
    m_coneLifterPID = m_conePickupLiftMotor.getPIDController();

    m_conePickupLiftMotor.setInverted(true);
    
    m_cubePickupEncoder = m_cubePickupMotor.getEncoder();
    m_coneLifterEncoder = m_conePickupLiftMotor.getEncoder();
    Shuffleboard.getTab("Tab5").addDouble("Cone Winch Encoder", ()-> m_coneLifterEncoder.getPosition());
    
    m_coneLifterPID.setFeedbackDevice(m_coneLifterEncoder);
    m_coneLifterPID.setP(IntakeConstants.CONE_LIFTER_P);
    m_coneLifterPID.setI(IntakeConstants.CONE_LIFTER_I);
    m_coneLifterPID.setD(IntakeConstants.CONE_LIFTER_D);

  }

  public void setCubePickupIntake(double pickupSpeed) {
    
    m_cubePickupMotor.set(pickupSpeed); 
    
  }

  public void setConePickupExtension(boolean deployed){
    m_coneExtendSolenoid.set(deployed);
  }


  public void setCubeExtend(double speed){
    m_cubeDeployMotor.set(speed);
    lastCubeDeploySpeed = speed;
  }

  public double getCubeExtender(){
    
    return lastCubeDeploySpeed;
  }

  public void setConeLifterPosition(double pos){
    m_coneLifterPID.setReference(-1*pos, CANSparkMax.ControlType.kPosition);
  }


  public double getCubePickupSpeed(){
    return m_cubePickupEncoder.getVelocity();
  }

}
