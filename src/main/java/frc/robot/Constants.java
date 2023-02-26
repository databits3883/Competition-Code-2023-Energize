// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
  
      public static final double ROTATION_GEARING = 1.0/12.8;
      public static final double VELOCITY_GEARING = 1.0/6.75;
      public static final double WHEEL_CIRCUMFRENCE = Math.PI * 4*2.54 *0.01;

      public static final double DRIVE_TRACK_WIDTH = 0.4826;
      public static final double DRIVE_TRACK_LENGTH = 0.6985;

      // Reading from offset on cancoders, absolute encoder is correctly zeroed now.
      public static final double FRONT_RIGHT_CALIBRATE_ENCODER_OFFSET = 0;//9
      public static final double BACK_RIGHT_CALIBRATE_ENCODER_OFFSET = 0;//12
      public static final double FRONT_LEFT_CALIBRATE_ENCODER_OFFSET = 0;//10
      public static final double BACK_LEFT_CALIBRATE_ENCODER_OFFSET = 0;//11

      //Old Comp Values
      // public static final double FRONT_RIGHT_CALIBRATE_ENCODER_OFFSET = 357.100;//9
      // public static final double BACK_RIGHT_CALIBRATE_ENCODER_OFFSET = 312.978;//12
      // public static final double FRONT_LEFT_CALIBRATE_ENCODER_OFFSET = 16.875;//10
      // public static final double BACK_LEFT_CALIBRATE_ENCODER_OFFSET = 127.090;//11

      // Locations for the swerve drive modules relative to the robot center.
      // Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
      // Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
      // Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
      // Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

      // // Creating my kinematics object using the module locations
      // SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      //   m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
      // );

      private static final Translation2d m_frontLeftLocation = new Translation2d(DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2);
      private static final Translation2d m_frontRightLocation = new Translation2d(DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2);
      private static final Translation2d m_backLeftLocation = new Translation2d(-DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2);
      private static final Translation2d m_backRightLocation = new Translation2d(-DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2);

      //public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
      public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(m_frontRightLocation, m_backRightLocation, m_backLeftLocation, m_frontLeftLocation);
      
      
      public static final double MAX_WHEEL_SPEED = 4.38;

      public static final double MAX_TURN_SPEED = MAX_WHEEL_SPEED * (Math.sqrt(DRIVE_TRACK_LENGTH*DRIVE_TRACK_LENGTH+DRIVE_TRACK_WIDTH*DRIVE_TRACK_WIDTH));

      public static final TrajectoryConstraint CONSTRAINT = new SwerveDriveKinematicsConstraint(KINEMATICS, MAX_WHEEL_SPEED);
      public static final TrajectoryConfig CONFIG = new TrajectoryConfig(MAX_WHEEL_SPEED,2);

      public static class CANChannels{
          public static final int FRONT_RIGHT_VELOCITY = 2;
          public static final int FRONT_RIGHT_ROTATION = 1;
          public static final int REAR_RIGHT_VELOCITY = 8;
          public static final int REAR_RIGHT_ROTATION = 7;
          public static final int REAR_LEFT_VELOCITY = 6;
          public static final int REAR_LEFT_ROTATION = 5;
          public static final int FRONT_LEFT_VELOCITY = 4;
          public static final int FRONT_LEFT_ROTATION = 3;

          public static final int FRONT_RIGHT_CALIBRATION = 9;
          public static final int REAR_RIGHT_CALIBRATION = 12;
          public static final int REAR_LEFT_CALIBRATION = 11;
          public static final int FRONT_LEFT_CALIBRATION = 10;
      }

      public static class CalibrationConstants{
          public static final double CALIBRATION_SPEED = -0.04;
          public static final int CALIBRATION_WAIT_MILLIS = 0;
          public static final int CALIBRATION_WAIT_NANOS = 10;

          //public static final double FRONT_RIGHT_SWITCH_LOCATION = -4.122;
          //public static final double REAR_RIGHT_SWITCH_LOCATION = -2.54;
          //public static final double REAR_LEFT_SWITCH_LOCATION = -1.03;
          //public static final double FRONT_LEFT_SWITCH_LOCATION = 0.598;

        }    

    public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
    }

  }

  public static final class ArmConstants {

    public static class ElbowMotorConstants {
      public static final double kP = 13;//8
      public static final double kI = 0;//0.01
      public static final double kD = 0;
      public static final double kIz = 0;
      public static final double kFF = 0.1;
      public static final double kMaxOutput = 0.4;
      public static final double kMinOutput = -1;
      
      public static final double CUBE_PLACE_PICKUP = 0.73;//rotations
      public static final double CUBE_PLACE_LOW = 0.5;
      public static final double CUBE_PLACE_HIGH = 0.5;

      public static final double CONE_PLACE_PICKUP = 0.73;//rotations
      public static final double CONE_PLACE_LOW = 0.45;
      public static final double CONE_PLACE_HIGH = 0.45;

      public static final double RAISE_DEADBAND = 0.025;
      public static final double LOWER_DEADBAND = -0.0125;

      public static final double JOG_ELBOW = 0.05;
    }

    public static class ElevatorMotorConstants {
      public static final double kP = 2.4;
      public static final double kI = 0;//1e-4
      public static final double kD = 0;
      public static final double kIz = 0;
      public static final double kFF = 0;
      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -1;

      public static final double TRAVEL = 4;

      public static final double CUBE_PLACE_PICKUP = 0;//in inches
      public static final double CUBE_PLACE_LOW = 6;//in inches
      public static final double CUBE_PLACE_HIGH = 20;//in inches

      public static final double CONE_PLACE_PICKUP = 0;//in inches
      public static final double CONE_PLACE_LOW = 6;//in inches
      public static final double CONE_PLACE_HIGH = 20;//in inches

      public static final double ENCODER_CONVERSION_FACTOR = 4.75/70;

    }

    public static class CANChannels{
      public static final int ELEVATOR = 13;
      public static final int ELBOW = 16;            
    }
    public static class PneumaticHubChannels {
      public static final int ARM_LIFT = 0;
      public static final int THE_CLAW = 1;
    }

    public static final class ArmLift {
      public static final boolean UP = false;
      public static final boolean DOWN = true;
    }
  }
  
  public static final class IntakeConstants {
    public static final class CANChannels {
      public static final int CONE_WINCH = 14;
      public static final int CUBE_PICKUP = 15;
      public static final int CUBE_EXTENDER = 17; /*main(dot)java*/
    }
    public static class PneumaticHubChannels {
      public static final int CONE_SPIKE = 2;
    }

    public static final double CONE_LIFTER_UP = 110;//arbitrayr
    public static final double CONE_LIFTER_DOWN = 0;//arbitrary

    public static final double CONE_LIFTER_P = 0.1;
    public static final double CONE_LIFTER_I = 0.00;
    public static final double CONE_LIFTER_D = 0.00;
  }

  public static final class GeneralConstants {
    public static final int PNEUMATIC_HUB_CAN_CHANNEL = 18;
    public static final int PNEUMATIC_HUB_PRESSURE_SENSOR_ID = 0;
  }
  
}
