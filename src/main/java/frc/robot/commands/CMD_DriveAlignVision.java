
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
import frc.robot.subsystems.Drive.SUB_Drivetrain;
//This primarly uses vision to align itself
import frc.robot.subsystems.Vision.SUB_Vision;
public class CMD_DriveAlignVision extends Command{
  private SUB_Drivetrain m_drivetrain;
  private SUB_Vision m_vision;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final PIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;
  private Pose2d goalPose;

  private CommandXboxController m_driverController;
//This only uses Odometry to align itself
  public CMD_DriveAlignVision(SUB_Drivetrain p_drivetrain, SUB_Vision p_vision, CommandXboxController p_driverController) {
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
    addRequirements(m_drivetrain);
    if (!m_vision.getHasLTarget() && !m_vision.getHasRTarget()){
      System.out.println("NOTHING SEEN DAWG");
      return;
    }
}

  @Override
  public void initialize() {
    System.out.println("Started Autoalign");
    goalPose = new Pose2d(0,0, new Rotation2d(0));
    end = false;

    /* Set the goals as an offset of the robot's current odometry */
    
    xController.setGoal(0);
    yController.setGoal(0);
    turnController.setSetpoint(0);

    xController.setTolerance(AutoAlignConstants.kXTolerance);
    yController.setTolerance(AutoAlignConstants.kYTolerance);
    turnController.setTolerance(AutoAlignConstants.kTurnTolerance);

    turnController.reset();

    turnController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    if (end) {
      return;
    }

    if (Math.abs(m_driverController.getLeftY()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getLeftX()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getRightX()) > AutoAlignConstants.kAbortThreshold) {
      end = true;
      System.out.println("Aborted by driver");
      return;
    }
    
    
    if (m_vision.getHasLTarget() && m_vision.getHasRTarget()){
      
    goalPose = new Pose2d(0,0, new Rotation2d(0));
    
    }else{
      turnSpeed = 0;
    }
    xController.setGoal(0);
    yController.setGoal(0+Units.inchesToMeters(0)); 
    xSpeed = MathUtil.clamp( xController.calculate(m_drivetrain.getTargetOdo().getX()), -AutoAlignConstants.kXAutoClamp, AutoAlignConstants.kXAutoClamp);
    ySpeed = MathUtil.clamp(yController.calculate(m_drivetrain.getTargetOdo().getY()), -AutoAlignConstants.kYAutoClamp, AutoAlignConstants.kYAutoClamp);  
    if (xController.atGoal()) {
      xSpeed = 0.0;
    }

    if (yController.atGoal()) {
      ySpeed = 0.0;
    }

    Logger.recordOutput("AutoAlignXSpeed", xSpeed);
    Logger.recordOutput("AutoAlignySpeed", ySpeed);
    Logger.recordOutput("AutoAlignTurnSpeed", turnSpeed);
    Logger.recordOutput("Autoalign turn Goal", goalPose.getRotation());
    Logger.recordOutput("YsetPoint", yController.getSetpoint().position);

    if (xController.atGoal() && yController.atGoal()) {
      System.out.println("At Goal " + Timer.getFPGATimestamp());
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