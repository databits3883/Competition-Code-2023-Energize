package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.CANChannels;
import frc.robot.Constants.ArmConstants.ElevatorMotorConstants;
import frc.robot.Constants.ArmConstants.PneumaticHubChannels;
import frc.robot.Constants.ArmConstants.ShoulderMotorConstants;

public class ArmSubsystem extends SubsystemBase {

    final CANSparkMax m_shoulder;
    final RelativeEncoder m_shoulderEncoder;
    final SparkMaxPIDController m_shoulderPidController;
  
    final CANSparkMax m_elevator;
    final RelativeEncoder m_elevatorEncoder;
    final SparkMaxPIDController m_elevatorPidController;

    final Solenoid m_armLift;
    final Solenoid m_theClaw;
     
      /** Creates a new Arm Subsystem. */
    public ArmSubsystem(PneumaticHub pneumaticHub) {
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
        m_elevatorPidController = m_elevator.getPIDController();
        m_elevatorPidController.setFeedbackDevice(m_elevatorEncoder);
        // set PID coefficients
        m_elevatorPidController.setP(ElevatorMotorConstants.kP);
        m_elevatorPidController.setI(ElevatorMotorConstants.kI);
        m_elevatorPidController.setD(ElevatorMotorConstants.kD);
        m_elevatorPidController.setIZone(ElevatorMotorConstants.kIz);
        m_elevatorPidController.setFF(ElevatorMotorConstants.kFF);
        m_elevatorPidController.setOutputRange(ElevatorMotorConstants.kMinOutput, ElevatorMotorConstants.kMaxOutput);

        m_armLift = pneumaticHub.makeSolenoid(PneumaticHubChannels.ARM_LIFT);
        m_theClaw = pneumaticHub.makeSolenoid(PneumaticHubChannels.THE_CLAW);

        Shuffleboard.getTab("Tab5").addDouble("Arm.ShoulderEncoder", ()->m_shoulderEncoder.getPosition());
        Shuffleboard.getTab("Tab5").addDouble("Arm.ElevatorEncoder", ()->m_elevatorEncoder.getPosition());
        Shuffleboard.getTab("Tab5").addBoolean("Arm.Lift", ()->m_armLift.get());
        Shuffleboard.getTab("Tab5").addBoolean("Arm.TheClaw.Grip", ()->m_theClaw.get());
    }

    public void setShoulderPosition(double position) {
        m_shoulderPidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }


    public void setArmLiftActive(boolean isActive) {
        m_armLift.set(isActive);
    }

    public void setTheClawGrip(boolean isGripping) {
        m_theClaw.set(isGripping);
    }     
}
