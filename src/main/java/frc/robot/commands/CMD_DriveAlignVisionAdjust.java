
//This only uses Odometry to align itself
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
//This primarly uses vision to align itself
import frc.robot.subsystems.Vision.SUB_Vision;
public class CMD_DriveAlignVisionAdjust extends Command{
  private SUB_Drivetrain m_drivetrain;
  private SUB_Vision m_vision;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final PIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;
  private double positionRatio;// its like a inverse cnc machine, the farther y the slower the x
  private double turnRatio;// its like a inverse cnc machine, the farther y the slower the turn
  private Pose2d goalPose;
  private Pose2d robotOdom;
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_setpoint;

  private double xAdjustment = 0;
  private double yAdjustment = 0;
  private double turnAdjustment = 0;

  private CommandXboxController m_driverController;
//This only uses Odometry to align itself
  public CMD_DriveAlignVisionAdjust(SUB_Drivetrain p_drivetrain, SUB_Vision p_vision, CommandXboxController p_driverController,
  double xAdjustment, double yAdjustment, double turnAdjustment) {
    m_drivetrain = p_drivetrain;
    m_vision = p_vision;
    m_driverController = p_driverController;

    xController = new ProfiledPIDController(
      Constants.AutoAlignConstants.driveKp,
      Constants.AutoAlignConstants.driveKi,
      Constants.AutoAlignConstants.driveKd,
      Constants.AutoAlignConstants.driveConstraints);

    yController = new ProfiledPIDController(
      Constants.AutoAlignConstants.driveKp,
      Constants.AutoAlignConstants.driveKi,
      Constants.AutoAlignConstants.driveKd,
      Constants.AutoAlignConstants.driveConstraints);

    turnController = new PIDController(
      Constants.AutoAlignConstants.turnKp,
      Constants.AutoAlignConstants.turnKi,
      Constants.AutoAlignConstants.turnKd);
      
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    this.xAdjustment = xAdjustment;
    this.yAdjustment = yAdjustment;
    this.turnAdjustment = turnAdjustment;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    System.out.println("Started Autoalign");
    // double GridAdjustment = (-(CurrentGrid - WantedGrid)*1.8);
    // Transform2d GridTransformation = new Transform2d(new Translation2d(0, GridAdjustment),new Rotation2d(0));
    goalPose = new Pose2d(0,0, new Rotation2d(0));
    // add back in when u have a variable that store this condition
    // if (m_variables.getHasCoral() == true){
      // goalPose = Constants.AutoAlignConstants.goalPose.get(m_variables.getAlignPosition());
    // }else{
      // goalPose = Constants.AutoAlignConstants.goalPose.get(m_variables.getAlignPosition());
    // }
    robotOdom = m_drivetrain.getPose();

    end = false;

    /* Set the goals as an offset of the robot's current odometry */
    xController.setGoal(0+Units.inchesToMeters(7.25)+xAdjustment);
    yController.setGoal(0+Units.inchesToMeters(0)+ yAdjustment);
  
    turnController.setSetpoint(0);

    xController.setTolerance(AutoAlignConstants.kXTolerance);
    yController.setTolerance(AutoAlignConstants.kYTolerance);
    turnController.setTolerance(AutoAlignConstants.kTurnTolerance);

    // xController.reset(m_drivetrain.getTargetOdo().getX());
    // yController.reset(m_drivetrain.getTargetOdo().getY());
    turnController.reset();

    turnController.enableContinuousInput(-180, 180);
    xController.setGoal(VisionConstants.kRobotToLCam.getX()+Units.inchesToMeters(7.25)+xAdjustment);
    yController.setGoal(0+Units.inchesToMeters(0)+ yAdjustment);
  }

  @Override
  public void execute() {
    xController.setGoal(VisionConstants.kRobotToLCam.getX()+Units.inchesToMeters(7.25)+xAdjustment);
    yController.setGoal(0+Units.inchesToMeters(0)+ yAdjustment);
  
    
    // if (m_vision.getHasTarget()){
    // turnRatio = m_vision.getTargetPose().getRotation().getAngle()/ m_drivetrain.getTargetOdo().getY();
    // }

    if (Math.abs(m_driverController.getLeftY()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getLeftX()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getRightX()) > AutoAlignConstants.kAbortThreshold) {
      end = true;
      System.out.println("Aborted by driver");
      return;
    }
    
    
    robotOdom = m_drivetrain.getPose();
    if (m_vision.getHasLTarget()&& m_vision.getHasRTarget()){
      // do error - navx = offset until its 0 camera 
      
    goalPose = new Pose2d(0,0, new Rotation2d(0));
    // goalPose = new Pose2d(goalPose.getX(), goalPose.getY(), new Rotation2d(goalPose.getRotation().getDegrees() - m_drivetrain.getTargetOdo().getRotation().getDegrees())); 
      // turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getAngle()), MathUtil.clamp(-0.1 * Math.abs(turnRatio), -.2, .2), MathUtil.clamp(-0.1 * Math.abs(turnRatio), -.2, .2));
      // if (Math.abs(m_drivetrain.getTargetOdo().getY()) <= .5){
        // turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getAngle()), -.2 , .2);
        // turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getTargetOdo().getRotation().rotateBy(new Rotation2d().fromDegrees(180)).getDegrees()), -.2 , .2);
      // }else{
      //   turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getAngle()), -.3 , .3);  
      // }
    
    }else{
      turnSpeed = 0;
    }
      xController.setGoal(0+Units.inchesToMeters(7.25)+xAdjustment);
      yController.setGoal(0+Units.inchesToMeters(0)+ yAdjustment);
    // if (m_vision.getHasTarget()){
      // if (Math.abs(m_vision.getTargetPose().getRotation().getAngle()) <= 5){  
        // xSpeed = MathUtil.clamp( xController.calculate(m_drivetrain.getTargetOdo().getX()), -AutoAlignConstants.kXAutoClamp, AutoAlignConstants.kXAutoClamp);
        // ySpeed = MathUtil.clamp(yController.calculate(m_drivetrain.getTargetOdo().getY()), -AutoAlignConstants.kYAutoClamp, AutoAlignConstants.kYAutoClamp);  
    
        // xSpeed = MathUtil.clamp( xController.calculate(m_drivetrain.getTargetOdo().getX()), MathUtil.clamp(-0.1 * Math.abs(positionRatio), -.3, .3), MathUtil.clamp(-0.1 * Math.abs(positionRatio), -.1, .1));
        // ySpeed = MathUtil.clamp(yController.calculate(m_drivetrain.getTargetOdo().getY()), -0.3, 0.3);  
    //   }else{
    //     xSpeed = MathUtil.clamp( xController.calculate(m_drivetrain.getTargetOdo().getX()), -0.1, 0.1);
    //     ySpeed = MathUtil.clamp(yController.calculate(m_drivetrain.getTargetOdo().getY()), -0.3, 0.3);  
    //   }
    // }else{
    //   xSpeed = MathUtil.clamp( xController.calculate(m_drivetrain.getTargetOdo().getX()), -0.2, 0.2);
    //   ySpeed = MathUtil.clamp(yController.calculate(m_drivetrain.getTargetOdo().getY()), -0.4, 0.4);  
    // }
    // }else{
    //   xController.setGoal(goalPose.getX()+2);
    //   yController.setGoal(goalPose.getY());
    //   xSpeed = xController.calculate(robotOdom.getX());
    //   ySpeed = yController.calculate(robotOdom.getY());
    // }
    if (xController.atGoal()) {
      xSpeed = 0.0;
    }

    if (yController.atGoal()) {
      ySpeed = 0.0;
    }

    // if (m_drivetrain.getTargetOdo().getX() <= Units.inchesToMeters(20)){
    //   m_drivetrain.setTargetOdoEnable(false);
    // }else{
      // m_drivetrain.setTargetOdoEnable(true);
    // }
    // turnSpeed = 0;  
    // turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getAngle()), -0.5, 0.5);
    // Logger.recordOutput("XError", m_drivetrain.getTargetOdo().getX());
    // Logger.recordOutput("YError", m_drivetrain.getTargetOdo().getY());
    // Logger.recordOutput("TurnError", m_drivetrain.getTargetOdo().getRotation().getDegrees());
    Logger.recordOutput("AutoAlignXSpeed", xSpeed);
    Logger.recordOutput("AutoAlignySpeed", ySpeed);
    Logger.recordOutput("AutoAlignTurnSpeed", turnSpeed);
    Logger.recordOutput("Autoalign turn Goal", goalPose.getRotation());
    // Logger.recordOutput("XCtrlOutput", yController.calculate(m_drivetrain.getTargetOdo().getY()));
    Logger.recordOutput("YsetPoint", yController.getSetpoint().position);

    if (xController.atGoal() && yController.atGoal()) {
      System.out.println("At Goal " + Timer.getFPGATimestamp());
      System.out.println("Goals" + "X=" + xController.getGoal().position + "Y=" + yController.getGoal().position);
      end = true;
      return;
    }         

    m_drivetrain.drive(xSpeed, ySpeed, -turnSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0.0, 0.0, 0.0, true);
    m_drivetrain.setTargetOdoEnable(true);
      
  }

  @Override
  public boolean isFinished() {
    if (end) {
      System.out.println("Done! " + Timer.getFPGATimestamp());
    }

    return end;
  }
}
