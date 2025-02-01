
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
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.Vision.SUB_Vision;
//This file uses odometery to adjust its location by the passed in doubles
public class CMD_DriveAdjustOdometery extends Command{
  private SUB_Drivetrain m_drivetrain;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final PIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;
  private Pose2d goalPose;
  private Pose2d robotOdom;
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_setpoint;

  private double xAdjustment = 0;
  private double yAdjustment = 0;
  private double turnAdjustment = 0;

  private CommandXboxController m_driverController;
//This only uses Odometry to align itself
  public CMD_DriveAdjustOdometery(SUB_Drivetrain p_drivetrain, SUB_Vision p_vision, CommandXboxController p_driverController
  , double xAdjustment, double yAdjustment, double turnAdjustment) {
    m_drivetrain = p_drivetrain;
    m_driverController = p_driverController;

    this.xAdjustment = xAdjustment;
    this.yAdjustment = yAdjustment;
    this.turnAdjustment = turnAdjustment; 

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
  
  }

  @Override
  public void initialize() {
    System.out.println("Started Autoalign");
    
    goalPose = new Pose2d(m_drivetrain.getPose().getX() + xAdjustment ,m_drivetrain.getPose().getY() + yAdjustment, m_drivetrain.getPose().getRotation().plus(new Rotation2d().fromDegrees(turnAdjustment)));
    
    robotOdom = m_drivetrain.getPose();

    end = false;

    /* Set the goals as an offset of the robot's current odometry */
    
    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
    turnController.setSetpoint(goalPose.getRotation().getDegrees());

    xController.setTolerance(AutoAlignConstants.kXTolerance);
    yController.setTolerance(AutoAlignConstants.kYTolerance);
    turnController.setTolerance(AutoAlignConstants.kTurnTolerance);

    xController.reset(m_drivetrain.getPose().getX());
    yController.reset(m_drivetrain.getPose().getY());
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
       
    turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getPose().getRotation().getDegrees()), -.2 , .2);
    // xSpeed = MathUtil.clamp( xController.calculate(m_drivetrain.getPose().getX()), -AutoAlignConstants.kXAutoClamp, AutoAlignConstants.kXAutoClamp);
    // ySpeed = MathUtil.clamp(yController.calculate(m_drivetrain.getPose().getY()), -AutoAlignConstants.kYAutoClamp, AutoAlignConstants.kYAutoClamp);
    
    xSpeed = MathUtil.clamp( xController.calculate(m_drivetrain.getPose().getX()), -.1, .1);
    ySpeed = MathUtil.clamp(yController.calculate(m_drivetrain.getPose().getY()), -.1, .1);  
    
    if (xController.atGoal()) {
      xSpeed = 0.0;
    }

    if (yController.atGoal()) {
      ySpeed = 0.0;
    }

    if (xController.atGoal() && yController.atGoal()) {
      System.out.println("At Goal " + Timer.getFPGATimestamp());
      end = true;
      return;
    }         

    m_drivetrain.drive(xSpeed, ySpeed, turnSpeed, false);

    Logger.recordOutput("Xgoal", xController.getGoal().position);
    
    Logger.recordOutput("Ygoal", yController.getGoal().position);
    Logger.recordOutput("turngoal", turnController.getSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    if (end) {
      System.out.println("Done! " + Timer.getFPGATimestamp());
    }

    return end;
  }
}
