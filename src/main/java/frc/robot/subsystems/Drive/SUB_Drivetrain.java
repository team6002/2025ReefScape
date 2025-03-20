// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Questimator.QuestNavIO;
import frc.robot.subsystems.Questimator.QuestimatorIOInputsAutoLogged;
import frc.robot.subsystems.Vision.SUB_Vision;
// import frc.robot.subsystems.SUB_Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.Constants.AutoConstants;

public class SUB_Drivetrain extends SubsystemBase {
      RobotConfig config;
      // Create MAXSwerveModules
      SwerveModule[] SwerveModules;
      private final SwerveModule m_frontLeft;
      private final SwerveModule m_frontRight;
      private final SwerveModule m_rearLeft; 
      private final SwerveModule m_rearRight;
      
      private final SwerveDrivePoseEstimator m_odometry;
      //Odometry that has nearest april tag as origin for use in autoalignment
      private final SwerveDrivePoseEstimator m_targetOdometry;
      private final SwerveDrivePoseEstimator questimetry;
      private final SwerveDriveOdometry m_pureOdometry;
      private SwerveModulePosition[] lastModulePositions = 
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
      // private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
    
      // private boolean onTarget = false;
      // private boolean onTargetV3 = false;
      // The gyro sensor
      // private final AHRS m_gyro = new AHRS(Port.kMXP);
      
      private final GyroIO gyroIO;
      private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
      
      private final QuestNavIO QuestNavIO;
      private final QuestimatorIOInputsAutoLogged questimatorInputs = new QuestimatorIOInputsAutoLogged();
      // Slew rate filter variables for controlling lateral acceleration
      // private double m_currentRotation = 0.0;
      // private double m_currentTranslationDir = 0.0;
      // private double m_currentTranslationMag = 0.0;
    
      // private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
      // private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
      // private double m_prevTime = WPIUtilJNI.now() * 1e-6;
      
      private Pose2d m_prevOdo = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
    
      private boolean TargetOdoEnable = true;
      // private Translation2d m_currentTarget = LocationConstants.SpeakerBlue;
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
      // private Pose2d currentPose;
      private int currentOdometry = 2; // 0 is just wheels, 1 is wheels and pv and 2 is questNav
  
      private Rotation2d m_cameraRotation;// angle of the robot from cameras
      public SUB_Drivetrain(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO
        ,SUB_Vision p_vision
        ,QuestNavIO p_questNavIO
        ) 
      {
            
        this.gyroIO = gyroIO;
        QuestNavIO = p_questNavIO;
        QuestNavIO.hardReset();
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
        // var targetStdDevs = VecBuilder.fill(0, 0, 0);
            
        
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
        
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }
    
        AutoBuilder.configure(
                this::getOdometry,
                // this::getPose, // Robot pose supplier
                // QuestNavIO::getRobotPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChasisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> driveAutoBuilder(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(AutoConstants.kPXController, 0.0, AutoConstants.kDXController), // Translation PID constants  
                        new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }else{
                    return false;
                  }
                },
                this // Reference to this subsystem to set requirements
        );
    
        m_vision = p_vision;
        m_odometry =
          new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(getAngle()),
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);
    
        m_targetOdometry =
          new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(getAngle()),
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);
        questimetry =
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
      public void setCurrentOdometry(int odo){
        currentOdometry = odo;
        System.out.println(odo);
      }

      public int getCurrentOdometry(){
        return currentOdometry;
      }

      public Pose2d getOdometry(){
        // Pose2d currentPose;
        if (currentOdometry == 2){
          // return QuestNavIO.getRobotPose();
          return questimetry.getEstimatedPosition();
        }if (currentOdometry == 1 ) {
          return m_odometry.getEstimatedPosition();
        }else{
          return m_pureOdometry.getPoseMeters();
        }
        // return currentPose;
      }

      @Override
      public void periodic() {
        SmartDashboard.putNumber("gyroHeading", getAngle());
        var RvisionEst = m_vision.getREstimatedGlobalPose();
        var LvisionEst = m_vision.getLEstimatedGlobalPose();
        m_vision.setRobotRotation(m_odometry.getEstimatedPosition().getRotation());
      
        // Update the odometry in the periodic block
        gyroIO.updateInputs(gyroInputs);
        QuestNavIO.updateInputs(questimatorInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        Logger.processInputs("Drive/QuestNav", questimatorInputs);
    
        QuestNavIO.cleanUpQuestNavMessages();
    
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
    
        m_targetOdometry.update(
          Rotation2d.fromDegrees(-getAngle()),
          // m_targetOdometry.getEstimatedPosition().getRotation(),
          getTargetModulePositions()
        );
        questimetry.update(
          QuestNavIO.getRobotPose().getRotation(),
           getModulePositions());
    
        m_pureOdometry.update(getOdoRotation(), modulePositions);
        Logger.recordOutput("PureRobotPose", m_pureOdometry.getPoseMeters());
        Logger.recordOutput("RobotPose",m_odometry.getEstimatedPosition());
        Logger.recordOutput("Questimetry", questimetry.getEstimatedPosition());
        // new Rotation2d();
        Logger.recordOutput("TargetOdometry",m_targetOdometry.getEstimatedPosition().rotateBy(Rotation2d.fromDegrees(180)));
        // new Rotation2d();
        // SmartDashboard.putBoolean("HasTarget", m_vision.getHasLTarget() || m_vision.getHasRTarget());    
        // SmartDashboard.putNumber("TargetYaw", getTargetOdo().getRotation().rotateBy(Rotation2d.fromDegrees(180)).getDegrees());
        m_vision.updateInputs();
        if (QuestNavIO.connected()){
          addQuestMeasurement(QuestNavIO.getRobotPose()
            ,Timer.getFPGATimestamp()-.04
            // ,QuestNavIO.timestamp()
          );
        }
        
        LvisionEst.ifPresent(
          est -> {
            // var estPose = m_vision.getLPose(m_odometry.getEstimatedPosition()).toPose2d();
            var estPose = est.estimatedPose.toPose2d();
            
              // estPose = m_vision.getEstimatedGlobalPose(estPose);
            // Logger.recordOutput("LCurrentPose", m_vision.getCurrentLPose());
            var estStdDevs = m_vision.getLEstimationStdDevs(estPose);
            // if (checkClosity(getPose(), est.estimatedPose.toPose2d())){
            //   estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            // }
            if (estStdDevs != VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)){
              Logger.recordOutput("LCameraPose", estPose);
            }
            // Change our trust in the measurement based on the tags we can see
            
              addVisionMeasurement(
                est.estimatedPose.toPose2d(), Timer.getFPGATimestamp() - .035, estStdDevs);
          }
        );
    
        RvisionEst.ifPresent(
          est -> {
            // var estPose = m_vision.getRPose(m_odometry.getEstimatedPosition()).toPose2d();
            var estPose = est.estimatedPose.toPose2d();
            // estPose = m_vision.getREstimatedGlobalPose();
            // estPose = m_vision.getEstimatedGlobalPose(estPose);
            // Logger.recordOutput("RCurrentPose", m_vision.getCurrentRPose());
            var estStdDevs = m_vision.getREstimationStdDevs(estPose);
            // if (checkClosity(getPose(), est.estimatedPose.toPose2d())){
            //   estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            // }
            if (estStdDevs != VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)){
              Logger.recordOutput("RCameraPose", estPose);
            }
            // Logger.recordOutput("RCameraPose", estPose);
            // Logger.recordOutput("REstimatePose", m_vision.getREstimatedGlobalPose());
            // Change our trust in the measurement based on the tags we can see
          
              addVisionMeasurement(
                est.estimatedPose.toPose2d(), Timer.getFPGATimestamp() - .035 , estStdDevs);
          
          }  
        );
       
        // if (LvisionEst.isPresent() && RvisionEst.isPresent()){
        //   try {
        //   var L = LvisionEst.get();
        //   var R = RvisionEst.get();
        //   var Rpose2d = RvisionEst.get().estimatedPose.toPose2d();
        //   var difPose = LvisionEst.get().estimatedPose.toPose2d().minus(Rpose2d);
          
          // if (
          //   Math.abs(difPose.getX()) < Units.inchesToMeters(3)
          //   &&
          //   Math.abs(difPose.getY()) < Units.inchesToMeters(3) 
          //   ){
          // System.out.println("GOOD");
          // var estStdDevs = m_vision.getREstimationStdDevs(L.estimatedPose.toPose2d());
             
        //     Matrix<N3, N1> stdDevs = VecBuilder.fill(0.25, 0.25, 0.25);
        //     addVisionMeasurement(
        //       L.estimatedPose.toPose2d(), L.timestampSeconds, stdDevs);
              
        //     addVisionMeasurement(
        //       R.estimatedPose.toPose2d(), R.timestampSeconds, stdDevs);
            
        //     }
        //   } catch(Exception e){

        //   }
        // }

        // LvisionEst.ifPresent(
        //   est -> {
        //       var estPose = est.estimatedPose.toPose2d();
        //     if (TargetOdoEnable){
        //       if (m_vision.getHasLTarget()){   
        //         addTargetVisionMeasurement(
        //           m_vision.getTargetLPose(), est.timestampSeconds);
        //       }
        //       if (m_vision.getHasRTarget()){
        //         addTargetVisionMeasurement(
        //           m_vision.getTargetRPose(), est.timestampSeconds);
        //         }
        //       }
        //   }  
        // );
        
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
      
      public boolean checkClosity(Pose2d robotPose, Pose2d CameraPose){
        var difPose = robotPose.minus(CameraPose);
        return ((Math.abs(difPose.getX()) < Units.inchesToMeters(12))) && Math.abs(difPose.getY()) < Units.inchesToMeters(12);
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
    
      public Pose2d getTargetOdo(){
        return m_targetOdometry.getEstimatedPosition();
      }
    
      /**
       * Resets the odometry to the specified pose.
       *
       * @param pose The pose to which to set the odometry.
       */
      public void resetOdometry(Pose2d pose) {
        setHeading(pose.getRotation().getDegrees());
        m_pureOdometry.resetPosition(
          Rotation2d.fromDegrees(getAngle()),
          getModulePositions(),
          pose
        );
        m_odometry.resetPosition(
          Rotation2d.fromDegrees(getAngle()),
          getModulePositions(),
          pose
        );
        questimetry.resetPosition(
          QuestNavIO.getRobotPose().getRotation(), 
          getModulePositions(), 
          pose);
      }
    
      public void resetTargetOdometry(Pose2d pose) {
        setHeading(pose.getRotation().getDegrees());
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(-getAngle()),
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
      public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                    Rotation2d.fromDegrees(getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
      }
      
      public void driveAutoBuilder(ChassisSpeeds p_ChassisSpeed){
          ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(p_ChassisSpeed, 0.02);
      
          SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
          setModuleStatesAuto(targetStates);
      }
    
      public double getXVelocity(){
        return getChasisSpeed().vxMetersPerSecond;
      }
      public double getYVelocity(){
        return getChasisSpeed().vyMetersPerSecond;
      }
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
    
      public void setModuleStatesAuto(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredStateAuto(desiredStates[0]);
        m_frontRight.setDesiredStateAuto(desiredStates[1]);
        m_rearLeft.setDesiredStateAuto(desiredStates[2]);
        m_rearRight.setDesiredStateAuto(desiredStates[3]);
      }
    
      //*Gets Rotation from  */
      public Rotation2d getOdoRotation(){
        return getPose().getRotation();
      }
    
      public void setPose(Pose2d pose){
        m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
      }
    
      // /** Zeroes the heading of the robot. LOL*/
      public void zeroHeading() {
        gyroIO.reset();
        // QuestNavIO.resetPose(new Pose2d(0, 0, new Rotation2d()));
        QuestNavIO.zeroHeading();
      }
    
      // public Command CMDzeroHeading() {
      //   return Commands.runOnce(()->zeroHeading(),this);
      // }
    
      public void setHeading(double p_DegAngle){
        // gyroIO.reset();
        gyroIO.set(Rotation2d.fromDegrees(p_DegAngle));
      }
    
      public void zeroOdometry(){
        resetOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
        // zeroHeading();
      
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
    
      public double getPitch(){
        return gyroInputs.pitch;
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
      //reverses the odometry for the targeting
      public SwerveModulePosition[] getTargetModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition (-m_frontLeft.getPosition().distanceMeters, m_frontLeft.getPosition().angle),
            new SwerveModulePosition (-m_frontRight.getPosition().distanceMeters, m_frontRight.getPosition().angle),
            new SwerveModulePosition (-m_rearLeft.getPosition().distanceMeters, m_rearLeft.getPosition().angle),
            new SwerveModulePosition (-m_rearRight.getPosition().distanceMeters, m_rearRight.getPosition().angle),
          };
      }
      
      // /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
      // public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
      //     m_odometry.addVisionMeasurement(visionMeasurement, timestampSeconds);
      // }
    
      /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
      public void addVisionMeasurement(
              Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
          //uses navx instead of camera vision.
          try {
          visionMeasurement.getRotation();
          m_cameraRotation = Rotation2d.k180deg;
          Pose2d p_angledPose = new Pose2d(visionMeasurement.getTranslation(), Rotation2d.fromDegrees(getAngle())); 
          m_odometry.addVisionMeasurement(p_angledPose, timestampSeconds, stdDevs);
          } catch (Exception e){
            System.out.println(e);
          }
      }
    
      public void addTargetVisionMeasurement(Transform3d visionMeasurement, double timestampSeconds) {
        try {
          Matrix<N3, N1> stdDevs = VecBuilder.fill(0.25, 0.25, 0.25);
          m_targetOdometry.addVisionMeasurement(new Pose2d(visionMeasurement.getX(), visionMeasurement.getY(), visionMeasurement.getRotation().toRotation2d()), timestampSeconds, stdDevs);
        } catch (Exception e){
          System.out.println(e);
        }
      }
      
      public void addQuestMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        try {
          Matrix<N3, N1> stdDevs = VecBuilder.fill(1, 1, 1);
          questimetry.addVisionMeasurement(new Pose2d(visionMeasurement.getX(), visionMeasurement.getY(), visionMeasurement.getRotation()), timestampSeconds, stdDevs);
        } catch (Exception e){
          System.out.println(e);
        }
      }
      // Create a list of waypoints from poses. Each pose represents one waypoint.
      // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
              new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
              new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
              new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
      );
    
      PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
      // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
    
      // Create the path using the waypoints created above
      PathPlannerPath path = new PathPlannerPath(
              waypoints,
              constraints,
              null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
              new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );
    
      public void setTargetOdoEnable(boolean state){
        TargetOdoEnable = state;
      }
    
      public void resetOdoToCurrentPosition(){
        try{
        
          resetOdometry(getPose());
          gyroIO.set(m_cameraRotation);
        } catch (Exception e) {
          DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }
      }
    
      /**
       * Resets the odometery to start of path
       * @return
       */
      public void resetOdoToStartPosition(String pathName){
        try{
          // Load the path you want to follow using its name in the GUI
          PathPlannerPath path;
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            path = PathPlannerPath.fromPathFile(pathName).flipPath();
          }else{
            path = PathPlannerPath.fromPathFile(pathName);
          }
          Optional<Pose2d> intialPose = path.getStartingHolonomicPose();
          intialPose.ifPresent(
            pose -> {
              System.out.println(pose.getRotation());
              gyroIO.set(pose.getRotation());
              resetOdometry(pose);
              // QuestNavIO.zeroHeading();
              QuestNavIO.resetPose(pose);
            }
            
          );
        } catch (Exception e) {
          DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }
      }
    /**
       * Resets the odometery to start of path
       * @return
       */
      public void resetOdoToStartPositionFlipped(String pathName){
        try{
          // Load the path you want to follow using its name in the GUI
          PathPlannerPath path;
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            path = PathPlannerPath.fromPathFile(pathName).flipPath().mirrorPath();
          }else{
            path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
          }
          Optional<Pose2d> intialPose = path.getStartingHolonomicPose();
          intialPose.ifPresent(
            pose -> {
              System.out.println(pose.getRotation());
              gyroIO.set(pose.getRotation());
              resetOdometry(pose);
              QuestNavIO.resetPose(pose);
            }
            
          );
        } catch (Exception e) {
          DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }
      }
    
      public Command FollowPath(String pathName) {
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            // PathPlannerTrajectory trajectory = path.generateTrajectory(getChasisSpeed(), getOdoRotation(), config);
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path).andThen(new InstantCommand(() -> drive(0, 0, 0, false),this));
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
      }
    
      public Command FollowPathFlipped(String pathName) {
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
            // PathPlannerTrajectory trajectory = path.generateTrajectory(getChasisSpeed(), getOdoRotation(), config);
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path).andThen(new InstantCommand(() -> drive(0, 0, 0, false),this));
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
      }

      public Command Stop(String pathName) {
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            // PathPlannerTrajectory trajectory = path.generateTrajectory(getChasisSpeed(), getOdoRotation(), config);
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
      }
      // Prevent the path from being flipped if the coordinates are already correct
      // path.preventFlipping = true;
      public void questNavReset(){
        QuestNavIO.hardReset();
      }
      
      public void setStartingAngle(){
        try {
          
        if (m_vision.getHasRTarget()){
          setHeading(Math.toDegrees(m_vision.getREstimatedGlobalPose().get().estimatedPose.getRotation().plus(new Rotation3d(Rotation2d.k180deg)).getAngle()));
        }else if (m_vision.getHasLTarget()){
          setHeading(Math.toDegrees(m_vision.getLEstimatedGlobalPose().get().estimatedPose.getRotation().plus(new Rotation3d(Rotation2d.k180deg)).getAngle()));
        }else{
        }
        } catch (Exception e) {
        }
      

      }
}