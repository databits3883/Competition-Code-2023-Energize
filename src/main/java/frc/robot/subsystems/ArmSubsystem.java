package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.CANChannels;
import frc.robot.Constants.DriveConstants.ShoulderMotorConstants;

public class ArmSubsystem extends SubsystemBase {

    final CANSparkMax m_shoulder;
    final RelativeEncoder m_shoulderEncoder;
    final SparkMaxPIDController m_shoulderPidController;
  
    final CANSparkMax m_elevator;
    final RelativeEncoder m_elevatorEncoder;
     
      /** Creates a new Arm Subsystem. */
    public ArmSubsystem() {
        m_shoulder = new CANSparkMax(CANChannels.SHOULDER, MotorType.kBrushed);
        m_shoulderEncoder = m_shoulder.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
        m_shoulderPidController = m_shoulder.getPIDController();
        m_shoulderPidController.setFeedbackDevice(m_shoulderEncoder);
        // set PID coefficients
        m_shoulderPidController.setP(ShoulderMotorConstants.kP);
        m_shoulderPidController.setI(ShoulderMotorConstants.kI);
        m_shoulderPidController.setD(ShoulderMotorConstants.kD);
        m_shoulderPidController.setIZone(ShoulderMotorConstants.kIz);
        m_shoulderPidController.setFF(ShoulderMotorConstants.kFF);
        m_shoulderPidController.setOutputRange(ShoulderMotorConstants.kMinOutput, ShoulderMotorConstants.kMaxOutput);


        m_elevator = new CANSparkMax(CANChannels.ELEVATOR, MotorType.kBrushless);
        m_elevatorEncoder = m_elevator.getEncoder();
    }

    public void setPosition(double position) {
        m_shoulderPidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }
     
}
