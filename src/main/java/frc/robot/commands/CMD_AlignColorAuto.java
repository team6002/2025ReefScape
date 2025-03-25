
//This only uses Odometry to align itself
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
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
  private Timer m_timer;
  private double m_turnTime;//how long we turn for

//This only uses Odometry to align itself
  public CMD_AlignColorAuto(SUB_Drivetrain p_drivetrain, SUB_Vision p_vision) {
    m_drivetrain = p_drivetrain;
    m_vision = p_vision;
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
    m_timer.start();
    System.out.println("Started Autoalign");
    end = false;

    turnController.setSetpoint(0);

    xController.setTolerance(AutoAlignConstants.kXTolerance);
    yController.setTolerance(AutoAlignConstants.kYTolerance);
    turnController.setTolerance(AutoAlignConstants.kTurnToleranceColor);

    turnController.reset();

    turnController.enableContinuousInput(-180, 180);
    if (m_vision.getTcameraYaw() != Double.MAX_VALUE){
      try { 
        m_turnTime = Math.abs(m_vision.getTcameraYaw() * .007); 
        turnSpeed = Math.copySign(.2, -m_vision.getTcameraYaw());
      } catch (Exception e) {
      }
    }else{
      m_turnTime = 0;
    }
  }

  @Override
  public void execute() {
    if (xController.atGoal()) {
      xSpeed = 0.0;
    }

    if (yController.atGoal()) {
      ySpeed = 0.0;
    }

    if (m_timer.get() >= m_turnTime){
      end = true;
    }

    Logger.recordOutput("AutoAlignTurnSpeed", turnSpeed);

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
