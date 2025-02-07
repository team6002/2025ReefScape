// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
    
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    // This accounts for stuff such as wheel wear//323 is the middle fo the field
    public static final double kXFactor = 0.96;  // if actual is smaller than odo go down  .96 is brand new // .912 baldest

    public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) * kXFactor; // meters
    public static final double kDrivingEncoderVelocityFactor = (((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0) * kXFactor; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.14;//0.004;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.21;

    public static final double kDrivingA = 0.0;
    public static final double kDrivingS = 0.125;
    public static final double kDrivingV = 2.4           ;
    
    // public static final double kDrivingA = 0.44218;
    // public static final double kDrivingS = 0.17491;
    // public static final double kDrivingV = 2.7538;

    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    // public static final double kTurningP = 1.5;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    public static final double kPAcceleration = 0.005;

    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class CoralHolderConstants{
    public static final double kP = 0.00001;
    public static final double kPLoaded = 0.0001;
    public static final double kI = 0.0;
    public static final double kILoaded = 0.0;
    public static final double kD = 0.0;
    public static final double kDLoaded = 0.0;
    public static final double kFF = 0.000175;
    public static final double kFFLoaded = 0.0025;
    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final boolean kCoralHolderInverted = true;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final double kIntake = 3000;
    public static final double kOff = 0;
    public static final double kReverse = -2500;
    public static final double kHolding = 50;
  }

  public static final class ElevatorPivotConstants{
    public static final double kP = 3.5;//2.0
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kS = 0.03;//.03
    public static final double kG = 0.025;//.025, .18
    public static final double kV = 4.45;//4.45
    public static final double kPivotOffset = Math.toRadians(-90);
    public static final double kMaxVel = Math.toRadians(700);//720
    public static final double kMaxAccel = Math.toRadians(700);//720
    public static final double kMaxVelExtended = Math.toRadians(540);
    public static final double kMaxAccelExtended = Math.toRadians(90);
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;
    public static final double kConversionFactor = 2*Math.PI;
    public static final double kTolerance = Math.toRadians(2);
    
    public static final double kHome = Math.toRadians(5);
    public static final double kReady = Math.toRadians(65);
    public static final double kReadyToScore = Math.toRadians(82);
    public static final double kIntake = Math.toRadians(65);
    public static final double kDeployl1 = Math.toRadians(82.75);
    public static final double kDeployl2 = Math.toRadians(86);
    public static final double kDeployl3 = Math.toRadians(88);//85.5
  }

  public static final class ElevatorConstants{
    public static final double kP = 0.02;//.075, .02
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kS = 0.2;//.233
    public static final double kV = 0.045;//.052
    public static final double kG = 0.3;//.01
    public static final double kMaxVel = 325;
    public static final double kMaxAccel = 325;
    public static final double kMaxVelDown = 100;
    public static final double kMaxAccelDown = 100;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;
    public static final double kConversionFactor = 3.135;//1.9006
    public static final double kTolerance = 2;
    public static final double kHome = 0;
    public static final double kReady = 14.5;
    public static final double kIntake = 14.5;
    public static final double kDeployL1 = 0;
    public static final double kDeployL2 = 28;
    public static final double kDeployL3 = 68;
  }

  public static final class WristConstants{
    public static final boolean kWristInverted = false;
    public static final double kWristOffset = -Math.PI;
    public static final double kP = 0.4;//.2
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kS = 0.0;//.1
    public static final double kG = 0.17;//.2
    public static final double kV = 1.15;//2.7, 1.15
    public static final double kMaxVel = Math.toRadians(540);//1080
    public static final double kMaxAccel = Math.toRadians(360);//1080
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final double kConverstionFactor = 2*Math.PI;
    public static final double kTolerance = Math.toRadians(3);

    public static final double kHome = Math.toRadians(110);
    public static final double kReady = Math.toRadians(-30);
    public static final double kStowing = Math.toRadians(0);
    public static final double kIntake = -Math.toRadians(110);
    public static final double kReadyToScore = Math.toRadians(-15);
    public static final double kDeployl1 = Math.toRadians(30);//35
    public static final double kDeployl2 = Math.toRadians(22.5);//20
    public static final double kDeployl3 = Math.toRadians(37.5);//30
  }

   public static final class LocationConstants{
    public static final Translation2d SpeakerBlue = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(218));
    public static final Translation2d SpeakerRed = new Translation2d(16.54, Units.inchesToMeters(218));
  
    public static final Translation2d SpeakerShootingBlue = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(218));
    public static final Translation2d SpeakerShootingRed = new Translation2d(16.54 - Units.inchesToMeters(0), Units.inchesToMeters(218));
    // the location in which we shoot at to stage it.
    public static final Pose2d StageBlue = new Pose2d( 0.9,6.5, Rotation2d.fromDegrees(-90));
    public static final Pose2d StageRed = new Pose2d( 15.8,6.5, Rotation2d.fromDegrees(-90));
    
    public static final Pose2d SubwooferBlue = new Pose2d( 1.2,5.4, Rotation2d.fromDegrees(0));
    public static final Pose2d AmpBlue = new Pose2d( 1.9,7.8, Rotation2d.fromDegrees(-90));
    public static final Pose2d LSourceBlue = new Pose2d( 15,.5, Rotation2d.fromDegrees(125));// the source closest to blue side
    
    public static final Pose2d SubwooferRed = new Pose2d( 15.4,5.4, Rotation2d.fromDegrees(-180));
    public static final Pose2d AmpRed = new Pose2d( 14.8,7.8, Rotation2d.fromDegrees(-90));
    public static final Pose2d LSourceRed = new Pose2d( 1.6,.5, Rotation2d.fromDegrees(60));// the source closest to blue side
    
  }

  public static final class VisionConstants{
    public static final String kFrontCameraName = "FrontCamera";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class HardwareConstants{
    // SPARK MAX CAN IDs 
    public static final int kFrontLeftDrivingCanId = 2;//8
    public static final int kFrontLeftTurningCanId = 1;//9
    public static final int kFrontRightDrivingCanId =  8;//2
    public static final int kFrontRightTurningCanId = 9;//1
    public static final int kRearLeftDrivingCanId = 19;//10
    public static final int kRearLeftTurningCanId = 5;//7
    public static final int kRearRightDrivingCanId = 10;//18
    public static final int kRearRightTurningCanId = 7;//3
    public static final int kCoralHolderCanId = 16;//11
    public static final int kLeftElevatorCanId = 18;//14
    public static final int kRightElevatorCanId = 17;//15
    public static final int kLeftPivotCanId = 3;//6
    public static final int kRightPivotCanId = 6;//4
    public static final int kWristCanId = 14;//13
    public static final int kClimbCanId = 4;
    public static final int kAlgaeCanId = 15;
  }

  public static final class AutoAlignConstants{
      /* X and Y drive constraints. Output ranges [-1, 1] */
      public static final double kXTolerance = 0.02;
      public static final double kYTolerance = 0.02;

      public static final double kXAutoClamp = .3;
      public static final double kYAutoClamp = .6;
      public static final double kTurnAutoClamp = .3;
  
      public static final TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(2, 1.5);
      public static final double driveKp = 1;
      public static final double driveKi = 0.;
      public static final double driveKd = 0.;
  
      /* Turn constraints. Output ranges [-1, 1] */
      public static final double kTurnTolerance = .5;
      public static final TrapezoidProfile.Constraints turnConstraints = new TrapezoidProfile.Constraints(1, .75);
      public static final double turnKp = 0.011;
      public static final double turnKi = 0.;
      public static final double turnKd = 0.;
  
      /* Absolute joystick threshold for driver abort */
      public static final double kAbortThreshold = 0.2;
   
  }
}
