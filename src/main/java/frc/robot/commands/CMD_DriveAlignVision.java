
//This only uses Odometry to align itself
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private Pose2d robotOdom;

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
  }

  @Override
  public void initialize() {
    // double GridAdjustment = (-(CurrentGrid - WantedGrid)*1.8);
    // Transform2d GridTransformation = new Transform2d(new Translation2d(0, GridAdjustment),new Rotation2d(0));

    // add back in when u have a variable that store this condition
    // if (m_variables.getHasCoral() == true){
      // goalPose = Constants.AutoAlignConstants.goalPose.get(m_variables.getAlignPosition());
    // }else{
      // goalPose = Constants.AutoAlignConstants.goalPose.get(m_variables.getAlignPosition());
    // }
    robotOdom = m_drivetrain.getPose();

    end = false;

    /* Set the goals as an offset of the robot's current odometry */
    xController.setGoal(0);
    yController.setGoal(0);
    // turnController.setSetpoint(goalPose.getRotation().getDegrees());

    xController.setTolerance(AutoAlignConstants.kXTolerance);
    yController.setTolerance(AutoAlignConstants.kYTolerance);
    turnController.setTolerance(AutoAlignConstants.kTurnTolerance);

    xController.reset(m_vision.getTargetYDistance());
    yController.reset(m_vision.getTargetYDistance());
    // turnController.reset();

    // turnController.enableContinuousInput(-180, 180);
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
    
    
    robotOdom = m_drivetrain.getPose();
    if (m_vision.getHasTarget()){
      xController.setGoal(0);
      yController.setGoal(0);
      xSpeed = xController.calculate(m_vision.getTargetXDistance());
      ySpeed = yController.calculate(m_vision.getTargetYDistance());
    }else{
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
      xSpeed = xController.calculate(robotOdom.getX());
      ySpeed = yController.calculate(robotOdom.getY());
    }
    if (xController.atGoal()) {
      xSpeed = 0.0;
    }

    if (yController.atGoal()) {
      ySpeed = 0.0;
    }

    turnSpeed = 0;  
    // turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getAngle()), -0.5, 0.5);

    SmartDashboard.putNumber("AutoAlignXSpeed: ", xSpeed);
    SmartDashboard.putNumber("AutoAlignYSpeed: ", ySpeed);
    SmartDashboard.putNumber("AutoAlignTurnSpeed: ", turnSpeed);
    SmartDashboard.putNumber("AutoAlignTurnGoal", turnController.getSetpoint());

    if (xController.atGoal() && yController.atGoal()) {
      System.out.println("At Goal " + Timer.getFPGATimestamp());
      end = true;
      return;
    }         

    m_drivetrain.drive(xSpeed, ySpeed, turnSpeed, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0.0, 0.0, 0.0, true, false);
  }

  @Override
  public boolean isFinished() {
    if (end) {
      System.out.println("Done! " + Timer.getFPGATimestamp());
    }

    return end;
  }
}
