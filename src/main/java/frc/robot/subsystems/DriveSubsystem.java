// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.DriveConstants.*;

import java.util.List;




public class DriveSubsystem extends SubsystemBase {

  boolean m_allCalibrated = false;

  public final SwerveDriveKinematics m_kinematics = KINEMATICS;
  final Module[] m_modules = new Module[4];
  final Pigeon2 m_gyro;
  
  // public final SwerveDriveKinematicsConstraint m_swerveConstraint = new SwerveDriveKinematicsConstraint ( m_kinematics, 0.25);
  // public final CentripetalAccelerationConstraint m_centripetalConstraint = new CentripetalAccelerationConstraint(0.2);
  // public final List Constraints = List.of(SwerveDriveKinematicsConstraint, CentripetalAccelerationConstraint);
  public final SwerveDriveOdometry m_odometry;
  SwerveModulePosition[] m_lastMeasuredPositions = new SwerveModulePosition[4];

  //private Pose2d m_relativePoseOffset = new Pose2d(1.65,0.508, Rotation2d.fromDegrees(180));
  private Pose2d m_startingPose = new Pose2d(0,0, Rotation2d.fromDegrees(180));

private final Field2d m_fieldTracker;

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    // m_kinematics = KINEMATICS;
    
    m_modules[0] = new Module(CANChannels.FRONT_RIGHT_VELOCITY, CANChannels.FRONT_RIGHT_ROTATION, CANChannels.FRONT_RIGHT_CALIBRATION,FRONT_RIGHT_CALIBRATE_ENCODER_OFFSET,"Front Right Module");
    m_modules[1] = new Module(CANChannels.REAR_RIGHT_VELOCITY, CANChannels.REAR_RIGHT_ROTATION, CANChannels.REAR_RIGHT_CALIBRATION,BACK_RIGHT_CALIBRATE_ENCODER_OFFSET,"Back Right Module");
    m_modules[2] = new Module(CANChannels.REAR_LEFT_VELOCITY, CANChannels.REAR_LEFT_ROTATION, CANChannels.REAR_LEFT_CALIBRATION,BACK_LEFT_CALIBRATE_ENCODER_OFFSET,"Back Left Module");
    m_modules[3] = new Module(CANChannels.FRONT_LEFT_VELOCITY, CANChannels.FRONT_LEFT_ROTATION, CANChannels.FRONT_LEFT_CALIBRATION,FRONT_LEFT_CALIBRATE_ENCODER_OFFSET,"Front Left Module");



    m_gyro = new Pigeon2(14,"rio");


    m_lastMeasuredPositions[0] = new SwerveModulePosition(0, new Rotation2d(0));
    m_lastMeasuredPositions[1] = new SwerveModulePosition(0, new Rotation2d(0));
    m_lastMeasuredPositions[2] = new SwerveModulePosition(0, new Rotation2d(0));
    m_lastMeasuredPositions[3] = new SwerveModulePosition(0, new Rotation2d(0));

    
    m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(m_gyro.getYaw()), m_lastMeasuredPositions, m_startingPose);
    measureCurrentPositions();
    m_odometry.resetPosition(Rotation2d.fromDegrees(m_gyro.getYaw()), m_lastMeasuredPositions, m_odometry.getPoseMeters().relativeTo(m_startingPose));
    

    

    Shuffleboard.getTab("Game Screen").addDouble("GyroYaw", () -> m_gyro.getYaw());
    Shuffleboard.getTab("Tab5").addDouble("GyroYaw", () -> m_gyro.getYaw());
    Shuffleboard.getTab("Tab5").addDouble("GyroPitch", () -> m_gyro.getPitch());
    Shuffleboard.getTab("Tab5").addDouble("GyroRoll", () -> m_gyro.getRoll());

    m_fieldTracker = new Field2d();
    addChild("Field Position",m_fieldTracker);
    setChassisSpeed(new ChassisSpeeds(0, 0, 0));

    resetGyro(0);
    
    SmartDashboard.putData(m_fieldTracker);


  }

  public void setSpeedFieldRelative(ChassisSpeeds speeds){
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
     speeds.omegaRadiansPerSecond, Rotation2d.fromDegrees(m_gyro.getYaw()));
     
     setChassisSpeed(speeds);
  }

  public void setSpeedFieldRelativePivot(ChassisSpeeds speeds, double povAngle){
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
     speeds.omegaRadiansPerSecond, Rotation2d.fromDegrees(m_gyro.getYaw()));
     
     setChassisSpeedWithPivot(speeds,povAngle);
  }

  public void setDisplayTrajectory(Trajectory t){
    m_fieldTracker.getObject("traj").setTrajectory(t);
  }

  public void setChassisSpeed(ChassisSpeeds speeds){
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    setStates(states);
  }

  public void setChassisSpeedWithPivot(ChassisSpeeds speeds, double povAngle){
    double xOffset = Math.sin(povAngle * (Math.PI / 180)) * (Constants.DriveConstants.DRIVE_TRACK_LENGTH/2 + 0.25);
    double yOffset = Math.cos(povAngle * (Math.PI / 180)) * (Constants.DriveConstants.DRIVE_TRACK_WIDTH/2 + 0.25);

    Translation2d offseTranslation = new Translation2d(xOffset,yOffset);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds, offseTranslation);
    setStates(states);
  }

  public void setStates(SwerveModuleState[] states){

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEED);
      
      for(int i=0;i<4;i++){
        m_modules[i].setPosition(states[i]);
      }
  }

  public void resetStates(){
    for(int i=0; i<4; i++){

      m_modules[i].setPosition(new SwerveModuleState(0,Rotation2d.fromDegrees(0)));
    }
  }

  public boolean getAllCalibrated(){
    return m_allCalibrated;
  }

  public void calibrate(double angle){
    for(Module m : m_modules){
      m.calibrate();
    }
    
    m_allCalibrated = true;

    resetGyro(angle);
  }


  public Pose2d getFieldPose(){
    return m_odometry.getPoseMeters().relativeTo(m_startingPose.times(-1));
  }

  public void setStartingPose(){
    m_startingPose = m_odometry.getPoseMeters();
    
  }

  public void visionUpdateFieldPose(Pose2d visionPose){
    m_odometry.resetPosition(Rotation2d.fromDegrees(-m_gyro.getYaw()), m_lastMeasuredPositions, visionPose.times(-1));
  }

  public void resetGyro(double angleOffset){

    m_gyro.setYaw(angleOffset);
    
    m_odometry.resetPosition(Rotation2d.fromDegrees(m_gyro.getYaw()), m_lastMeasuredPositions, m_odometry.getPoseMeters());

  }

  public double getGyroRoll(){
    return m_gyro.getRoll();
  }

  public double getGyroYaw(){
    return m_gyro.getYaw();
  }

  public Pose2d getCurrentPoseEstimate(){
    return m_odometry.getPoseMeters();
  }

  

  void measureCurrentPositions(){
    for(int i=0;i<4;i++){
      m_lastMeasuredPositions[i] = m_modules[i].measurePosition();

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measureCurrentPositions();
    m_odometry.update(Rotation2d.fromDegrees(m_gyro.getYaw()+180), m_lastMeasuredPositions);


    m_fieldTracker.setRobotPose(getFieldPose());
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

    private String m_moduleName;

    private CANSparkMax m_velocityMotor;
    private RelativeEncoder m_velocityEncoder;
    private SparkMaxPIDController m_velocityController;

    private CANCoder m_calibrateEncoder;
    private double m_calibrationOffset;

    private double lastAngleSP = 0;
    private double lastSpeedSP =0;

    Module(int velocityChannel,int rotationChannel, int calibrationChannel, double calibrationOffset, String moduleName){

      m_moduleName = moduleName;
      m_rotationMotor = new CANSparkMax(rotationChannel, MotorType.kBrushless);
      m_rotationEncoder = m_rotationMotor.getEncoder();
      m_rotationController = m_rotationMotor.getPIDController();


      m_rotationMotor.setInverted(false);


      m_rotationEncoder.setPositionConversionFactor(ROTATION_GEARING);
      m_rotationController.setFeedbackDevice(m_rotationEncoder);

      m_rotationController.setP(6);//0.6
      m_rotationController.setI(0.001);
      m_rotationController.setD(0);
      m_rotationController.setFF(0);

      
      m_rotationController.setPositionPIDWrappingMaxInput(Rotation2d.fromDegrees(360).getRotations());
      m_rotationController.setPositionPIDWrappingMinInput(Rotation2d.fromDegrees(0).getRotations());
      m_rotationController.setPositionPIDWrappingEnabled(true);

      m_velocityMotor = new CANSparkMax(velocityChannel, MotorType.kBrushless);
      m_velocityEncoder = m_velocityMotor.getEncoder();
      m_velocityController = m_velocityMotor.getPIDController();

      m_velocityMotor.setInverted(false);

      m_velocityEncoder.setVelocityConversionFactor(VELOCITY_GEARING*WHEEL_CIRCUMFRENCE * (1.0/60.0));
      m_velocityEncoder.setPositionConversionFactor(VELOCITY_GEARING*WHEEL_CIRCUMFRENCE*Math.PI*2);

      m_velocityController.setP(0.22);
      m_velocityController.setI(0);
      m_velocityController.setD(1.2);
      m_velocityController.setFF(0.23);




      m_calibrateEncoder = new CANCoder(calibrationChannel);
      System.out.println("Before reset" + m_calibrateEncoder.getAbsolutePosition());


      //m_rotationEncoder.setPosition((m_calibrateEncoder.getAbsolutePosition() + m_calibrateEncoder.configGetMagnetOffset())/360);
      m_calibrationOffset = calibrationOffset;
      calibrate();
      Shuffleboard.getTab("Tab5").addDouble(m_moduleName + " Calibrate Encoder", () -> m_calibrateEncoder.getAbsolutePosition());
      Shuffleboard.getTab("Tab5").addDouble(m_moduleName + " Rotation Encoder", () -> m_rotationEncoder.getPosition()*360);
      
 
    }

    
    public SwerveModuleState measureState(){
      return new SwerveModuleState(m_velocityEncoder.getVelocity(),new Rotation2d(m_rotationEncoder.getPosition()));
    }

    public SwerveModulePosition measurePosition() {
      return new SwerveModulePosition(
          m_velocityEncoder.getPosition(), Rotation2d.fromRotations(m_rotationEncoder.getPosition()));
    }

    public void setPosition(SwerveModuleState state){

      state = SwerveModuleState.optimize(state, Rotation2d.fromRotations(m_rotationEncoder.getPosition()));
      
      if(state.speedMetersPerSecond != lastSpeedSP){
        m_velocityController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        lastSpeedSP = state.speedMetersPerSecond;
      }

      double angle = state.angle.getRotations();

      if(angle != lastAngleSP){

        m_rotationController.setReference(angle,ControlType.kPosition);
        lastAngleSP = angle;
      }
    }

    public void calibrate(){

      
      m_rotationEncoder.setPosition(((m_calibrateEncoder.getAbsolutePosition() - m_calibrationOffset)% 360) / 360);
      System.out.println("After Reset " + ((m_calibrateEncoder.getAbsolutePosition() - m_calibrationOffset)% 360) / 360);
      m_rotationController.setReference(0, ControlType.kPosition);

    }

    

  }
}