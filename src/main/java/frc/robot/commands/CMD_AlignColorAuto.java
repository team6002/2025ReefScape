
//This only uses Odometry to align itself
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
//This primarly uses vision to align itself
import frc.robot.subsystems.Vision.SUB_Vision;
public class CMD_AlignColorAuto extends Command{
  private SUB_Drivetrain m_drivetrain;
  private SUB_Vision m_vision;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final PIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;
  // private double positionRatio;// its like a inverse cnc machine, the farther y the slower the x
  // private double turnRatio;// its like a inverse cnc machine, the farther y the slower the turn
  private Pose2d goalPose;
  private Pose2d robotOdom;
  private Timer m_timer;
  private double m_turnTime;//how long we turn for
  // private TrapezoidProfile.State m_goal;
  // private TrapezoidProfile.State m_setpoint;

  private double xAdjustment = 0;
  private double yAdjustment = 0;
  private double turnAdjustment = 0;

  private CommandXboxController m_driverController;
//This only uses Odometry to align itself
  public CMD_AlignColorAuto(SUB_Drivetrain p_drivetrain, SUB_Vision p_vision) {
    m_drivetrain = p_drivetrain;
    m_vision = p_vision;
    // m_driverController = p_driverController;
    m_turnTime = 0;
    m_timer = new Timer();

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
      0.0005,
      Constants.AutoAlignConstants.turnKi,
      Constants.AutoAlignConstants.turnKd);
      
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.stop();
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

    turnController.setSetpoint(0);

    xController.setTolerance(AutoAlignConstants.kXTolerance);
    yController.setTolerance(AutoAlignConstants.kYTolerance);
    turnController.setTolerance(AutoAlignConstants.kTurnToleranceColor);

    // xController.reset(m_drivetrain.getTargetOdo().getX());
    // yController.reset(m_drivetrain.getTargetOdo().getY());
    turnController.reset();

    turnController.enableContinuousInput(-180, 180);
    
    if (Math.abs(m_vision.getTcameraYaw()) <= 3.5){
      end = true;
    }
  }

  @Override
  public void execute() {

    // xController.setGoal(VisionConstants.kRobotToLCam.getX()+Units.inchesToMeters(7.25)+xAdjustment);
    // yController.setGoal(0+Units.inchesToMeters(0)+ yAdjustment);
  
    
    // if (m_vision.getTcameraYaw() != Double.MAX_VALUE){
    //   try {    
    //     // turnSpeed = MathUtil.clamp(turnController.calculate(m_vision.getTcameraYaw()), -.1, .1);
    //   } catch (Exception e) {
    //   }
    // }

    if (m_vision.getTcameraYaw() != Double.MAX_VALUE){
    try { 
        m_timer.stop();
        turnSpeed = Math.copySign(.1, -m_vision.getTcameraYaw());
        // turnSpeed = MathUtil.clamp(turnController.calculate(m_vision.getTcameraYaw()), -.1, .1);
    } catch (Exception e) {
    }
    }else{
      m_timer.start();
      if (m_timer.get() >= .8){
        turnSpeed = .1;
      }else {
        if (m_timer.get() >= 1.6){
          m_timer.reset();
        }
        turnSpeed = -.1;
      }
      // m_turnTime = 0;
    }

    // if (Math.abs(m_driverController.getLeftY()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getLeftX()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getRightX()) > AutoAlignConstants.kAbortThreshold) {
    //   end = true;
    //   System.out.println("Aborted by driver");
    //   return;
    // }
    
    if (xController.atGoal()) {
      xSpeed = 0.0;
    }

    if (yController.atGoal()) {
      ySpeed = 0.0;
    }

    if (Math.abs(m_vision.getTcameraYaw()) <= 3.5){
      end = true;
    }
    // turnSpeed = 0;  
    // turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getAngle()), -0.5, 0.5);
    // Logger.recordOutput("XError", m_drivetrain.getTargetOdo().getX());
    // Logger.recordOutput("YError", m_drivetrain.getTargetOdo().getY());
    // Logger.recordOutput("TurnError", m_drivetrain.getTargetOdo().getRotation().getDegrees());
    // Logger.recordOutput("AutoAlignXSpeed", xSpeed);
    // Logger.recordOutput("AutoAlignySpeed", ySpeed);
    Logger.recordOutput("AutoAlignTurnSpeed", turnSpeed);
    // Logger.recordOutput("Autoalign turn Goal", goalPose.getRotation());
    // Logger.recordOutput("XCtrlOutput", yController.calculate(m_drivetrain.getTargetOdo().getY()));
    // Logger.recordOutput("YsetPoint", yController.getSetpoint().position);


    m_drivetrain.drive(xSpeed, ySpeed, turnSpeed, false);
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
