// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;




public class DriveSubsystem extends SubsystemBase {

  boolean m_allCalibrated = false;

  final SwerveDriveKinematics m_kinematics;
  final Module[] m_modules = new Module[4];
  final AHRS m_gyro;


  final SwerveDriveOdometry m_odometry;
  SwerveModulePosition[] m_lastMeasuredPositions = new SwerveModulePosition[4];

  private Pose2d m_relativePoseOffset = new Pose2d();

private final Field2d m_fieldTracker;

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    m_kinematics = KINEMATICS;
    
    m_modules[0] = new Module(CANChannels.FRONT_RIGHT_VELOCITY, CANChannels.FRONT_RIGHT_ROTATION, CANChannels.FRONT_RIGHT_CALIBRATION);
    m_modules[1] = new Module(CANChannels.REAR_RIGHT_VELOCITY, CANChannels.REAR_RIGHT_ROTATION, CANChannels.REAR_RIGHT_CALIBRATION);
    m_modules[2] = new Module(CANChannels.REAR_LEFT_VELOCITY, CANChannels.REAR_LEFT_ROTATION, CANChannels.REAR_LEFT_CALIBRATION);
    m_modules[3] = new Module(CANChannels.FRONT_LEFT_VELOCITY, CANChannels.FRONT_LEFT_ROTATION, CANChannels.FRONT_LEFT_CALIBRATION);

    m_gyro = new AHRS(I2C.Port.kMXP,(byte)200);
    System.out.println("did odometry");
    //m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(),m_lastMeasuredPositions);
    m_lastMeasuredPositions[0] = new SwerveModulePosition(0, new Rotation2d(0));
    m_lastMeasuredPositions[1] = new SwerveModulePosition(0, new Rotation2d(0));
    m_lastMeasuredPositions[2] = new SwerveModulePosition(0, new Rotation2d(0));
    m_lastMeasuredPositions[3] = new SwerveModulePosition(0, new Rotation2d(0));
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), m_lastMeasuredPositions);
    //m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());
    m_fieldTracker = new Field2d();
    addChild("Field Position",m_fieldTracker);
  }

  public void setSpeedFieldRelative(ChassisSpeeds speeds){
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
     speeds.omegaRadiansPerSecond, Rotation2d.fromDegrees(-m_gyro.getYaw()));
     setChassisSpeed(speeds);
  }

  public void setDisplayTrajectory(Trajectory t){
    m_fieldTracker.getObject("traj").setTrajectory(t);
  }

  public void setChassisSpeed(ChassisSpeeds speeds){
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    setStates(states);
  }
  public void setStates(SwerveModuleState[] states){
    if(m_allCalibrated){
      m_kinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEED);
      //m_kinematics. (states, MAX_WHEEL_SPEED);
      for(int i=0;i<4;i++){
        m_modules[i].setPosition(states[i]);
      }
    }
  }

  public boolean getAllCalibrated(){
    return m_allCalibrated;
  }

  public void calibrate(){
    for(Module m : m_modules){
      m.calibrate();
    }
    m_allCalibrated = true;
  }


  public Pose2d getPoseRelative(){
    return m_odometry.getPoseMeters().relativeTo(m_relativePoseOffset);
  }

  public void setPoseRelative(){
    m_relativePoseOffset = m_odometry.getPoseMeters();
  }

  public void resetGyro(){
    //m_gyro.reset();
    m_gyro.zeroYaw();
    m_odometry.resetPosition(m_gyro.getRotation2d(), m_lastMeasuredPositions, m_relativePoseOffset);
  }

  public void setGyroAngleAdjustment(double angle){
    m_gyro.setAngleAdjustment(angle);
  }

  public Pose2d getCurrentPoseEstimate(){
    return m_odometry.getPoseMeters();
  }

  

  void measureCurrentPositions(){
    for(int i=0;i<4;i++){
      m_lastMeasuredPositions[i] = m_modules[i].measurePosition();
      //System.out.println("settin da position to: " + m_modules[i].measurePosition());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measureCurrentPositions();
    m_odometry.update(Rotation2d.fromDegrees(-m_gyro.getAngle()), m_lastMeasuredPositions);

    m_fieldTracker.setRobotPose(getPoseRelative());
  }


  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("X location", ()->m_odometry.getPoseMeters().getX(), null);
    builder.addDoubleProperty("Y location", ()->m_odometry.getPoseMeters().getY(), null);
  }

  private class Module{
    private CANSparkMax m_rotationMotor;
    private RelativeEncoder m_rotationEncoder;
    private SparkMaxPIDController m_rotationController;

    private CANSparkMax m_velocityMotor;
    private RelativeEncoder m_velocityEncoder;
    private SparkMaxPIDController m_velocityController;

    private CANCoder m_calibrateEncoder;

    private double lastAngleSP = 0;
    private double lastSpeedSP =0;

    Module(int velocityChannel,int rotationChannel, int calibrationChannel){
      m_rotationMotor = new CANSparkMax(rotationChannel, MotorType.kBrushless);
      m_rotationEncoder = m_rotationMotor.getEncoder();
      m_rotationController = m_rotationMotor.getPIDController();


      //m_rotationMotor.setInverted(true);
      m_rotationMotor.setInverted(false);

      // m_rotationEncoder.setInverted(true);
      m_rotationEncoder.setPositionConversionFactor(ROTATION_GEARING * Math.PI*2);
      m_rotationController.setFeedbackDevice(m_rotationEncoder);

      m_rotationController.setP(0.6);
      m_rotationController.setI(0);
      m_rotationController.setD(0);
      m_rotationController.setFF(0);

      m_velocityMotor = new CANSparkMax(velocityChannel, MotorType.kBrushless);
      m_velocityEncoder = m_velocityMotor.getEncoder();
      m_velocityController = m_velocityMotor.getPIDController();

      m_velocityMotor.setInverted(false);

      m_velocityEncoder.setVelocityConversionFactor(VELOCITY_GEARING*WHEEL_CIRCUMFRENCE * (1.0/60.0));

      m_velocityController.setP(0.22);
      m_velocityController.setI(0);
      m_velocityController.setD(1.2);
      m_velocityController.setFF(0.23);


      m_calibrateEncoder = new CANCoder(calibrationChannel);
      Shuffleboard.getTab("Tab 5").addNumber("Can bus voltage "+m_rotationMotor.getDeviceId(), m_calibrateEncoder::getBusVoltage);
      
      // Shuffleboard.getTab("Tab 5").addNumber("Rotation encoder "+m_rotationMotor.getDeviceId(), m_rotationEncoder::getPosition);
      // Shuffleboard.getTab("Tab 5").addNumber("Calibrate encoder "+m_rotationMotor.getDeviceId(), m_calibrateEncoder::getAbsolutePosition);
    }

    /* 
    public SwerveModuleState measureState(){
      return new SwerveModuleState(m_velocityEncoder.getVelocity(),new Rotation2d(m_rotationEncoder.getPosition()));
    }*/

    public SwerveModulePosition measurePosition() {
      return new SwerveModulePosition(
          m_velocityEncoder.getPosition(), new Rotation2d(m_rotationEncoder.getPosition()));
    }

    public void setPosition(SwerveModuleState state){

      state = SwerveModuleState.optimize(state, new Rotation2d(m_rotationEncoder.getPosition()));
      
      if(state.speedMetersPerSecond != lastSpeedSP){
        m_velocityController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        lastSpeedSP = state.speedMetersPerSecond;
      }

      double angle = mapAngleToNearContinuous(state.angle.getRadians());
      if(angle != lastAngleSP){
        m_rotationController.setReference(angle,ControlType.kPosition);
        lastAngleSP = angle;
      }
    }

    public void calibrate(){
      m_rotationEncoder.setPosition(m_calibrateEncoder.getAbsolutePosition()*Math.PI/180.0);
      //System.out.println("Calibrated wheel"+m_rotationMotor.getDeviceId()+" to "+m_rotationEncoder.getPosition());
    }

    

    double mapAngleToNearContinuous(double newAngle){
      double currentAngle = m_rotationEncoder.getPosition();
      long completedRotations = Math.round(currentAngle / (Math.PI*2));
      double offsetAngle = newAngle%(Math.PI*2) + (2*Math.PI*completedRotations);
      if(Math.abs(currentAngle - offsetAngle) < Math.PI){
          return offsetAngle;
      }else if(offsetAngle > currentAngle){
          return offsetAngle - Math.PI*2;
      }else{
          return offsetAngle + Math.PI*2;
      }
  }


    


    

  }
}
