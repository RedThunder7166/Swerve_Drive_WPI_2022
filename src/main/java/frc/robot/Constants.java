// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kRearLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kRearRightDriveMotorPort = 6;
    
        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kRearLeftTurningMotorPort = 3;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kRearRightTurningMotorPort = 7;
    
        public static final int kFrontLeftTurningEncoderPorts = 9; 
        public static final int kRearLeftTurningEncoderPorts = 10;
        public static final int kFrontRightTurningEncoderPorts = 11;
        public static final int kRearRightTurningEncoderPorts = 12;

        public static final double kFrontLeftAngleZero = 353.1; // FIXME: Add angle offset
        public static final double kRearLeftAngleZero = 78.8; // FIXME: Add angle offset
        public static final double kFrontRightAngleZero = 331.6; // FIXME: Add angle offset
        public static final double kRearRightAngleZero = 323.7; // FIXME: Add angle offset
    
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kRearRightTurningEncoderReversed = true;
    
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kRearRightDriveEncoderReversed = true;
    
        public static final double kTrackWidth = 0.5969; // meters
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.5969; // meters
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
        public static final boolean kGyroReversed = false;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.509;
        public static final double kvVoltSecondsPerMeter = 2.73;
        public static final double kaVoltSecondsSquaredPerMeter = 0.124;
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxRotationalSpeedMetersPerSecond = 4; // Constant multiplied by controller input

        public static final double ksTurning = 0.742; // FIXME feedforward turning
        public static final double kvTurning = 0.216;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;

      }
    
      public static final class ModuleConstants {
        // Drive motor -> FX Encoder (2048 units)
        // Turning motor -> CTRE CANcoder (4096 units)
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 20 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 35 * Math.PI;
        public static final double kDriveGearRatio = 8.14;
        public static final double kTurningGearRatio = 12.8;

        public static final int kDriveFXEncoderCPR = 2048;
        public static final int kTurningCANcoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.1016; // 4 inches
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // C = D * pi
        public static final double kDrivetoMetersPerSecond = (10 * kWheelCircumferenceMeters)/(kDriveGearRatio * 2048);

        // Unused in our code

        // public static final double kDriveEncoderDistancePerPulse =
        //     // Assumes the encoders are directly mounted on the wheel shafts
        //     (kWheelDiameterMeters * Math.PI) / (double) kDriveCANcoderCPR;
    
        // public static final double kTurningEncoderDistancePerPulse =
        //     // Assumes the encoders are on a 1:1 reduction with the module shaft.
        //     (2 * Math.PI) / (double) kTurningFalconEncoderCPR;
    
        public static final double kPModuleTurningController = 7.5; // FIXME kp Turning
        public static final double kDModuleTurningController = .1; // FIXME kD Turning
    
        public static final double kPModuleDriveController = 3; // FIXME kp driving
      }
    
      public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
		    public static final int kOperatorControllerPort = 1;
      }
    
      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 2;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
      }

      public static final class MechanismConstants{

        public static final int kIntakeMotor = 0;  // FIXME change this
        public static final int kIndexerMotor1 = 1; //FIXME change this
        public static final int kIndexerMotor2 = 2; //FIXME change this
        public static final int kExteriorClimb = 0; //FIXME change this 
        public static final int kInteriorClimb = 0; //FIXME change this
        public static final int kInteriorTrack = 0; //FIXME change this
        public static final int kExteriorTrack = 0; //FIXME change this 
      }
    }