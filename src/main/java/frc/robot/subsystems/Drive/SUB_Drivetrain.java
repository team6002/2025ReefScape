// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LocationConstants;
import frc.robot.subsystems.Vision.SUB_Vision;
import frc.robot.subsystems.Vision.VisionIO;
// import frc.robot.subsystems.SUB_Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class SUB_Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  // SUB_Vision m_vision;
  SwerveModule[] SwerveModules;
  // Module[] swerveModules = new Module[4];
  private final SwerveModule m_frontLeft;
      // = new SwerveModule(
      // HardwareConstants.kFrontLeftDrivingCanId,
      // HardwareConstants.kFrontLeftTurningCanId,
      // DriveConstants.kFrontLeftChassisAngularOffset,
      // "FrontLeft"
      // );

  private final SwerveModule m_frontRight;
      // = new SwerveModule(
      // HardwareConstants.kFrontRightDrivingCanId,
      // HardwareConstants.kFrontRightTurningCanId,
      // DriveConstants.kFrontRightChassisAngularOffset,
      // "FrontRight"
      // );

  private final SwerveModule m_rearLeft; 
      // = new SwerveModule(
      // HardwareConstants.kRearLeftDrivingCanId,
      // HardwareConstants.kRearLeftTurningCanId,
      // DriveConstants.kBackLeftChassisAngularOffset,
      // "BackLeft"
      // );

  private final SwerveModule m_rearRight;
      // = new SwerveModule(
      // HardwareConstants.kRearRightDrivingCanId,
      // HardwareConstants.kRearRightTurningCanId,
      // DriveConstants.kBackRightChassisAngularOffset,
      // "BackRight"
      // );
  
  private final SwerveDrivePoseEstimator m_odometry;
  private final SwerveDriveOdometry m_pureOdometry;
  private SwerveModulePosition[] lastModulePositions = 
  new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

  private boolean onTarget = false;
  private boolean onTargetV3 = false;
  // The gyro sensor
  // private final AHRS m_gyro = new AHRS(Port.kMXP);
  
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  
  private Pose2d m_prevOdo = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));

  private Translation2d m_currentTarget = LocationConstants.SpeakerBlue;
  // Odometry class for tracking robot pose using only encoders

  // Available paths in teleop.  Will select path based on alliance color.
  public enum TeleopPath {
    AMP,
    SOURCE
  }

  Field2d field;
  Field2d fieldEst;
  /** Creates a new DriveSubsystem. */
  SUB_Vision m_vision;
  public SUB_Drivetrain(
    GyroIO gyroIO,
    ModuleIO flModuleIO,
    ModuleIO frModuleIO,
    ModuleIO blModuleIO,
    ModuleIO brModuleIO,
    SUB_Vision p_vision
    ) 
  {
        
    this.gyroIO = gyroIO;
    m_vision = p_vision;

    m_frontLeft = new SwerveModule(
      flModuleIO,
      0,
      DriveConstants.kFrontLeftChassisAngularOffset);
    m_frontRight = new SwerveModule(
      frModuleIO,
      1,
      DriveConstants.kFrontRightChassisAngularOffset);
    m_rearLeft = new SwerveModule(
      blModuleIO,
      2,
      DriveConstants.kBackLeftChassisAngularOffset);
    m_rearRight = new SwerveModule(
      brModuleIO, 
      3,
      DriveConstants.kBackRightChassisAngularOffset);
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);    
    
    SwerveModules = new SwerveModule[]{
      m_frontLeft,
      m_frontRight,
      m_rearLeft,
      m_rearRight
    };
    field = new Field2d();
    fieldEst = new Field2d();
    // m_ChassisSpeed = new ChassisSpeeds(0, 0, 0);
    // SmartDashboard.putNumber("SwerveP", m_SwerveP);
    // SmartDashboard.putNumber("SwerveI", m_SwerveI);
    // SmartDashboard.putNumber("SwerveD", m_SwerveD);
    // SmartDashboard.putNumber("SwerveFF", m_SwerveFF);
    // Configure AutoBuilder last
      // AutoBuilder.configureHolonomic(
      //         this::getPose, // Robot pose supplier
      //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      //         this::getChasisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      //         this::driveAutoBuilder, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
      //                 new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
      //                 new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
      //                 4, // Max module speed, in m/s
      //                 Units.inchesToMeters(11), // Drive base radius in meters. Distance from robot center to furthest module.
      //                 new ReplanningConfig() // Default path replanning config. See the API for the options here
      //         ),
      //         () -> {
      //             // Boolean supplier that controls when the path will be mirrored for the red alliance
      //             // This will flip the path being followed to the red side of the field.
      //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      //             return false;
      //         },
      //         this // Reference to this subsystem to set requirements
      // );

    // m_vision = p_vision;
    m_odometry =
      new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getAngle()),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);
    m_pureOdometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(getAngle()),
    getModulePositions()
    );
    
    m_vision.updateInputs();
    
  }


  
  // private double m_SwerveP = m_frontLeft.getSwerveP();
  // private double m_SwerveI = m_frontLeft.getSwerveI();
  // private double m_SwerveD = m_frontLeft.getSwerveD();
  // private double m_SwerveFF = m_frontLeft.getSwerveFF();

  @Override
  public void periodic() {
    // var visionEst = m_vision.getEstimatedGlobalPose();
  
    // Update the odometry in the periodic block
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    

    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();
    if (DriverStation.isDisabled()){
      Logger.recordOutput("SwerveStates/setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
    }

    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

    
    m_odometry.update(
      Rotation2d.fromDegrees(getAngle()),
      // getModulePositions()
      modulePositions
    );
    m_pureOdometry.update(getOdoRotation(), modulePositions);
    Logger.recordOutput("PureRobotPose", m_pureOdometry.getPoseMeters());
    Logger.recordOutput("RobotPose",m_odometry.getEstimatedPosition());

    

    // visionEst.ifPresent(
    //   est -> {
    //       var estPose = est.estimatedPose.toPose2d();
    //       Logger.recordOutput("CameraPose", estPose);
    //       // Change our trust in the measurement based on the tags we can see
    //       var estStdDevs = m_vision.getEstimationStdDevs(estPose);

    //       addVisionMeasurement(
    //         est.estimatedPose.toPose2d(), est.timestampSeconds);
    //   }  
    // );

    // if (visionEst.isPresent()){
      
    //   SmartDashboard.putNumber("targetYaw", m_vision.getTargetYaw());
    //   SmartDashboard.putNumber("EstX", Units.metersToInches(visionEst.get().estimatedPose.getX()));
    //   SmartDashboard.putNumber("EstY", Units.metersToInches(visionEst.get().estimatedPose.getY()));
    //   SmartDashboard.putNumber("EstDeg", Math.toDegrees(visionEst.get().estimatedPose.getRotation().getAngle()));
    //   // SmartDashboard.putNumber("targetAng", m_vision.getTargetYaw(7));
    //   // fieldEst.setRobotPose(visionEst.get().estimatedPose.toPose2d());
    // }
    
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++){
      moduleDeltas[moduleIndex] =
        new SwerveModulePosition(
          modulePositions[moduleIndex].distanceMeters
            - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }
    
  }

  public ChassisSpeeds getChasisSpeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }
  
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    // Pose2d p_decompPose = m_odometry.getEstimatedPosition();
    // Pose2d p_Pose2d = new Pose2d(p_decompPose.getX(), p_decompPose.getY(), p_decompPose.getRotation());
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    setHeading(pose.getRotation().getDegrees());
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getAngle()),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 600.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
  }
  
  public void driveAutoBuilder(ChassisSpeeds p_ChassisSpeed){
    // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(p_ChassisSpeed, 0.02);
    
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(new ChassisSpeeds(p_ChassisSpeed.vxMetersPerSecond, p_ChassisSpeed.vyMetersPerSecond, -p_ChassisSpeed.omegaRadiansPerSecond), 0.02);
    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    
    setModuleStates(targetStates);
  }


  /**
   * Using the PathPlanner pathfinding algorithm, pathfind from our current position to a path. Used
   * in teleop to pathfind to the start of a known path location.  Requires AutoPathBuilder to be
   * configured before use.  
   * @param wanted_path Path we want to pathfind to.  Known location in TeleopPath.
   * @return Command to follow the path that it found.
   */
  // public Command teleopPathfindTo(TeleopPath wanted_path){
  //   PathPlannerPath path;
  //   if (DriverStation.getAlliance().isPresent()){
  //     switch (wanted_path) {
  //       case AMP:
  //         if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
  //           path = PathPlannerPath.fromPathFile("RedAmp");
  //         }
  //         else {
  //           path = PathPlannerPath.fromPathFile("BlueAmp");
  //         }
  //         break;
  //       case SOURCE:
  //         if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
  //           path = PathPlannerPath.fromPathFile("RedSource");
  //         }
  //         else {
  //           path = PathPlannerPath.fromPathFile("BlueSource");
  //         }
  //         break;
        
  //       default:
  //         // no valid path to select.  Do nothing
  //         return new InstantCommand();
  //     }
  //   }else {
  //     // Driver alliance not selected
  //     return new InstantCommand();
  //   }
     
    
    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
  //   PathConstraints constraints = new PathConstraints(
  //           3.0, 2.0,
  //           Units.degreesToRadians(360), Units.degreesToRadians(180));
    
  //   Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
  //     path, 
  //     constraints,
  //     3.0 // Rotation delay in meters.  How far robot will travel before rotating.
  //     );
  //   return pathfindingCommand;
  // }

  // public void setModuleStates(SwerveModuleState[] desireStates){
  //   SwerveDriveKinematics.desaturateWheelSpeeds(
  //     desireStates, DriveConstants.kMaxSpeedMetersPerSecond);
  // }
  public double getXVelocity(){
    return getChasisSpeed().vxMetersPerSecond;
  }
  public double getYVelocity(){
    return getChasisSpeed().vyMetersPerSecond;
  }

  // public void runVelocity(ChassisSpeeds speeds){
  //   ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
  //   SwerveModuleState[] setpointStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
  //   SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSecond);
  //   SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
  //   for (int i = 0; i < 4; i++){
  //     optimizedSetpointStates[i] = swerveModules[i].runSetpoint(setpointStates[i]);
  //   }
  // }

  // public void stop(){
  //   runVelocity(new ChassisSpeeds());
  // }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  @AutoLogOutput (key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++ ){
      states[i] = SwerveModules[i].getState();
    }
    return states;
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  // private SwerveModulePosition[] getModuleStates(){
  //   SwerveModulePosition[] states = new SwerveModulePosition[4];
  //   for (int i = 0; i < 4; i++){
  //     states[i] = swerveModules[i].getPosition();
  //   }
  //   return states;
  // }
  
  //*Gets Rotation from  */
  public Rotation2d getOdoRotation(){
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose){
    m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }
  /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  //   m_frontLeft.resetEncoders();
  //   m_rearLeft.resetEncoders();
  //   m_frontRight.resetEncoders();
  //   m_rearRight.resetEncoders();
  // }

  /** Zeroes the heading of the robot. LOL*/
  public void zeroHeading() {
    // m_gyro.resetDisplacement();
    //*TODO fix */
  }

  public Command CMDzeroHeading() {
    return Commands.runOnce(()->zeroHeading(),this);
  }

  public void setHeading(double p_DegAngle){
    //* TODO FIX */
    // m_gyro.reset();
    // m_angleOffset = p_DegAngle;
  }

  public void zeroOdometry(){
    resetOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
    zeroHeading();
    // resetEncoders();
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  // public double getHeading() {
  //   return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  // }

  

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //TODO
    return gyroInputs.yawVelocity * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  public double getAngle() {
    // return Math.toDegrees(MathUtil.angleModulus(-Rotation2d.fromDegrees(m_gyro.getAngle()).getRadians())) + m_angleOffset;
    return gyroInputs.yawPosition.getDegrees();
  }

  public Rotation2d getRotation2d() {
    return gyroInputs.yawPosition;
    // return m_gyro.getRotation2d().plus(new Rotation2d(m_angleOffset));
  }
  //checks to see if the odometry is similiar enough to the vision
  public boolean getStableOdometry(){
    var p_OdoError = getPose().minus(m_prevOdo);
    if (Math.abs(p_OdoError.getX()) <= 0.25 && Math.abs(p_OdoError.getY()) <= 0.25){
    m_prevOdo = getPose();
      return true;
    }else {
    m_prevOdo = getPose();
      return false;
    }
  }


  /**
  * Get the SwerveModulePosition of each swerve module (position, angle). The returned array order
  * matches the kinematics module order.
  */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
      m_odometry.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
  public void addVisionMeasurement(
          Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
      //uses navx instead of camera vision.
      Pose2d p_angledPose = new Pose2d(visionMeasurement.getTranslation(), Rotation2d.fromDegrees(getAngle())); 
      m_odometry.addVisionMeasurement(p_angledPose, timestampSeconds, stdDevs);
  }

  
} 