// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.CacheRequest;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  final CANSparkMax m_cubePickupMotor;
  final CANSparkMax m_cubeDeployMotor;
  final CANSparkMax m_conePickupLiftMotor;
  final PneumaticHub m_pneumaticHub;
  final Solenoid m_coneExtendSolenoid;

  
  public Intake(int cubeIntakeMotorChannel, int cubeIntakeDeployMotorChannel, int coneLifterChannel, int coneExtendChannel, PneumaticHub PneumaticHub) {
    m_cubePickupMotor = new CANSparkMax(cubeIntakeMotorChannel, MotorType.kBrushless);
    m_cubeDeployMotor = new CANSparkMax(cubeIntakeDeployMotorChannel, MotorType.kBrushed);
    m_conePickupLiftMotor = new CANSparkMax(coneLifterChannel, MotorType.kBrushless);
    m_coneExtendSolenoid = new Solenoid(PneumaticsModuleType.REVPH, coneExtendChannel);

    m_pneumaticHub = PneumaticHub;


  }

  public void setCubePickupIntake(double pickupSpeed) {
    // This method will be called once per scheduler run
    m_cubePickupMotor.set(pickupSpeed); 
    
  }

  public void toggleConePickupExtension(){
    m_coneExtendSolenoid.toggle();
    
  }

  public void raiseCone(){
    m_conePickupLiftMotor.set(1);
  }

  public void lowerCone(){
    m_conePickupLiftMotor.set(-1);
  }


}
