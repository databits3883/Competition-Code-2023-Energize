package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.CANChannels;

public class ArmSubsystem extends SubsystemBase {

    final CANSparkMax m_shoulder;
    final RelativeEncoder m_shoulderEncoder;
    final CANSparkMax m_elevator;
    final RelativeEncoder m_elevatorEncoder;
     
      /** Creates a new Arm Subsystem. */
      public ArmSubsystem() {
        m_shoulder = new CANSparkMax(CANChannels.SHOULDER, MotorType.kBrushed);
        m_elevator = new CANSparkMax(CANChannels.ELEVATOR, MotorType.kBrushless);
        m_shoulderEncoder = m_shoulder.getEncoder();
        m_elevatorEncoder = m_elevator.getEncoder();
      }  
    
}
