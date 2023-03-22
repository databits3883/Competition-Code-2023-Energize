// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SignalLights extends SubsystemBase {
  final DigitalOutput coneInput = new DigitalOutput(21);
  final DigitalOutput cubeInput = new DigitalOutput(22);
  /** Creates a new SignalLights. */
  public SignalLights() {
    //coneInput.pulse(0);
  }

  public void ShowCube(){
    cubeInput.set(!cubeInput.get());
    
  }

  public void ShowCone(){
    
    coneInput.set(!coneInput.get());
  }

  
}
