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
      public static final double VELOCITY_GEARING = 1.0/5.14;
      public static final double WHEEL_CIRCUMFRENCE = Math.PI * 4*2.54 *0.01;

      public static final double DRIVE_TRACK_WIDTH = 0.43;
      public static final double DRIVE_TRACK_LENGTH = 0.73;

      public static final double FRONT_RIGHT_CALIBRATE_ENCODER_OFFSET = 318.780;//9
      public static final double BACK_RIGHT_CALIBRATE_ENCODER_OFFSET = 19.512;//12
      public static final double FRONT_LEFT_CALIBRATE_ENCODER_OFFSET = 214.629;//10
      public static final double BACK_LEFT_CALIBRATE_ENCODER_OFFSET = 143.174;//11

      public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
          new Translation2d(DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2), //front right 
          new Translation2d(-DRIVE_TRACK_LENGTH/2, -DRIVE_TRACK_WIDTH/2), //rear right
          new Translation2d(-DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2), //rear left
          new Translation2d(DRIVE_TRACK_LENGTH/2, DRIVE_TRACK_WIDTH/2) //front left
        );
      
      
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
    public static class ShoulderMotorConstants {
      public static final double kP = 0.1;
      public static final double kI = 1e-4;
      public static final double kD = 1;
      public static final double kIz = 0;
      public static final double kFF = 0;
      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -1;
      
      public static final double PICKUP = 0;
      public static final double PLACE_LOW = 50;
      public static final double PLACE_HIGH = 100;
    }

    public static class ElevatorMotorConstants {
      public static final double kP = 0.1;
      public static final double kI = 1e-4;
      public static final double kD = 1;
      public static final double kIz = 0;
      public static final double kFF = 0;
      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -1;
    }

    public static class CANChannels{
      public static final int ELEVATOR = 13;
      public static final int SHOULDER = 16;            
    }
    public static class PneumaticHubChannels{
      public static final int THE_CLAW = 2;
    }
  }
  
  public static final class IntakeConstants {
    
    public static final int CONE_LIFTER = 14;
    public static final int CUBE_LIFTER = 15;
    public static final int INTAKE_EXTENDER = 17;
    public static final int CONE_EXTENDER = 0;
  }

  public static final class GeneralConstants {
    public static final int PNEUMATIC_HUB = 18;
  }
  
}
