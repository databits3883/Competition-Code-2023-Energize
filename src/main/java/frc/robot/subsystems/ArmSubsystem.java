package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.CANChannels;
import frc.robot.Constants.ArmConstants.ElevatorMotorConstants;
import frc.robot.Constants.ArmConstants.PneumaticHubChannels;
import frc.robot.Constants.ArmConstants.ElbowMotorConstants;

public class ArmSubsystem extends SubsystemBase {

    final CANSparkMax m_elbow;
    final AbsoluteEncoder m_elbowEncoder;
    final PIDController m_elbowPidController;

  
    final CANSparkMax m_elevator;
    final RelativeEncoder m_elevatorEncoder;
    final SparkMaxPIDController m_elevatorPidController;

    final Solenoid m_armLift;
    final Solenoid m_theClaw;

    public enum ReachPosition {
        TRAVEL,
        
        CUBE_PICKUP,
        CUBE_LOW,
        CUBE_HIGH,
        
        CONE_PICKUP,
        CONE_LOW,
        CONE_HIGH;
    }
     
      /** Creates a new Arm Subsystem. */
    public ArmSubsystem(PneumaticHub pneumaticHub) {
        m_elbow = new CANSparkMax(CANChannels.ELBOW, MotorType.kBrushed);
        m_elbowEncoder = m_elbow.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_elbowPidController = new PIDController(ElbowMotorConstants.kP, ElbowMotorConstants.kI, ElbowMotorConstants.kD);
        
        // set PID coefficients
        m_elbowPidController.setP(ElbowMotorConstants.kP);
        m_elbowPidController.setI(ElbowMotorConstants.kI);
        m_elbowPidController.setD(ElbowMotorConstants.kD);
        m_elbowPidController.setTolerance(0.00001);
        m_elbow.setInverted(false);
        m_elbowEncoder.setInverted(false);
        


        m_elevator = new CANSparkMax(CANChannels.ELEVATOR, MotorType.kBrushless);
        m_elevatorEncoder = m_elevator.getEncoder();
        m_elevatorEncoder.setPositionConversionFactor(ElevatorMotorConstants.ENCODER_CONVERSION_FACTOR);
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

        Shuffleboard.getTab("Tab5").addDouble("Angle SP", () -> m_elbowPidController.getSetpoint());
        Shuffleboard.getTab("Tab5").addDouble("Arm.ElbowEncoder", ()->m_elbowEncoder.getPosition());
        Shuffleboard.getTab("Tab5").addDouble("Arm.ElevatorEncoder", ()->m_elevatorEncoder.getPosition());
        Shuffleboard.getTab("Tab5").addBoolean("Arm.Lift.down", ()->m_armLift.get());
        Shuffleboard.getTab("Tab5").addBoolean("Arm.TheClaw.grip", ()->m_theClaw.get());
        //Shuffleboard.getTab("Tab5").add(m_elbowPidController);
    }

    public void setElbowPosition(double position) {
        m_elbowPidController.setSetpoint(position);
        
    }

    public void changeElbowPosiiton(double delta){
        double changed = m_elbowEncoder.getPosition() + delta;

        m_elbowPidController.setSetpoint(changed);
    }

    public void runElbowPositionControl(){
        double output = m_elbowPidController.calculate(m_elbowEncoder.getPosition());

        output = Math.max(output, ElbowMotorConstants.kMinOutput);//does the MIN
        output = Math.min(output, ElbowMotorConstants.kMaxOutput);//does the MAX

        double error = m_elbowEncoder.getPosition() - m_elbowPidController.getSetpoint();
        if(error > ElbowMotorConstants.LOWER_DEADBAND && error < ElbowMotorConstants.RAISE_DEADBAND){
            m_elbowPidController.setSetpoint(m_elbowEncoder.getPosition());
            m_elbow.set(0);
        }
        else{
            m_elbow.set(-output);
        }

    }
    
    public boolean atElbowSetpoint(){
        return m_elbowPidController.atSetpoint();
    }

    public double getElbowEncoder(){
        return m_elbowEncoder.getPosition();
    }


    public void setElevatorPosition(double position) {
        m_elevatorPidController.setReference(position, ControlType.kPosition);
    }


    public void setArmLiftDown(boolean isDown) {
        m_armLift.set(isDown);
    }

    public void setTheClawGrip(boolean isGripping) {
        m_theClaw.set(isGripping);
    }     
}
