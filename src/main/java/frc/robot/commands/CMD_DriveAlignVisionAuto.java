
//This only uses Odometry to align itself
package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
//This primarly uses vision to align itself
import frc.robot.subsystems.Vision.SUB_Vision;
public class CMD_DriveAlignVisionAuto extends Command{
  private SUB_Drivetrain m_drivetrain;
  private SUB_Vision m_vision;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final PIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;
  private double xGoal, yGoal, turnGoal;
  private boolean turnComplete;

//This only uses Odometry to align itself
  public CMD_DriveAlignVisionAuto(SUB_Drivetrain p_drivetrain, SUB_Vision p_vision
  , double x, double y, double rot) {
    m_drivetrain = p_drivetrain;
    m_vision = p_vision;
  
    xGoal = x;
    yGoal = y;
    turnGoal = rot;
  
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
    
    // goalPose = new Pose2d(Units.inchesToMeters(11.5 + xGoal), Units.inchesToMeters(yGoal), new Rotation2d(0 + turnGoal));
    end = false;

    /* Set the goals as an offset of the robot's current odometry */
    
    xController.setGoal(Units.inchesToMeters(11 + xGoal));
    yController.setGoal(Units.inchesToMeters(yGoal));
    turnController.setSetpoint(0);

    xController.setTolerance(AutoAlignConstants.kXTolerance);
    yController.setTolerance(AutoAlignConstants.kYTolerance);
    turnController.setTolerance(AutoAlignConstants.kTurnTolerance);

    turnController.reset();

    turnController.enableContinuousInput(0, 360);
    turnComplete = false;
  
  }

  @Override
  public void execute() {
    if (end) {
      return;
    }
    var targOdo = m_drivetrain.getTargetOdo();
    
    
    // if (m_vision.getHasMTarget()){
    // if (!turnComplete){
    turnSpeed = 
      MathUtil.clamp(turnController.calculate(targOdo.getRotation().getDegrees()),-AutoAlignConstants.kTurnAutoClamp,AutoAlignConstants.kTurnAutoClamp);
    // }

    // }else{
    //   turnSpeed = 0;
    // }""
    System.out.println(turnController.getError());
    
    if (Math.abs(turnController.getError()) < 1){
      turnComplete = true;
      MathUtil.clamp(turnController.calculate(targOdo.getRotation().getDegrees()),-AutoAlignConstants.kFinishAutoClamp,AutoAlignConstants.kFinishAutoClamp);
     
    }

    xSpeed = MathUtil.clamp(xController.calculate(targOdo.rotateBy(Rotation2d.k180deg).getX()), -AutoAlignConstants.kXAutoClamp, AutoAlignConstants.kXAutoClamp);
    ySpeed = MathUtil.clamp(yController.calculate(targOdo.rotateBy(Rotation2d.k180deg).getY()), -AutoAlignConstants.kYAutoClamp, AutoAlignConstants.kYAutoClamp);  
    
    if (Math.abs(turnController.getError()) < 5){
    }else{
      xSpeed = MathUtil.clamp(xController.calculate(targOdo.rotateBy(Rotation2d.k180deg).getX()), -AutoAlignConstants.kFinishAutoClamp, AutoAlignConstants.kFinishAutoClamp);
      ySpeed = MathUtil.clamp(yController.calculate(targOdo.rotateBy(Rotation2d.k180deg).getY()), -AutoAlignConstants.kFinishAutoClamp, AutoAlignConstants.kFinishAutoClamp);  
    } 

    if (xController.atGoal()) {
      xSpeed = MathUtil.clamp(xController.calculate(targOdo.rotateBy(Rotation2d.k180deg).getX()), -AutoAlignConstants.kFinishAutoClamp, AutoAlignConstants.kFinishAutoClamp);
    }

    if (yController.atGoal()) {
      ySpeed = MathUtil.clamp(yController.calculate(targOdo.rotateBy(Rotation2d.k180deg).getY()), -AutoAlignConstants.kFinishAutoClamp, AutoAlignConstants.kFinishAutoClamp);  
    }


  
    
    Logger.recordOutput("AutoAlign/XSpeed", xSpeed);
    Logger.recordOutput("AutoAlign/YSpeed", ySpeed);
    Logger.recordOutput("AutoAlign/TurnSpeed", turnSpeed);
    Logger.recordOutput("AutoAlign/XSetpoint", xController.getSetpoint().position);
    Logger.recordOutput("AutoAlign/YSetpoint", yController.getSetpoint().position);
    Logger.recordOutput("AutoAlign/XGoal", xController.getGoal().position);
    Logger.recordOutput("AutoAlign/YGoal", yController.getGoal().position);
    // Logger.recordOutput("AutoAlign/turnSetpoint", turnSpeed);
    Logger.recordOutput("AutoAlign/TurnError", turnController.getError());
    Logger.recordOutput("AutoAlign/", targOdo);

    if (xController.atGoal() && yController.atGoal()) {
      System.out.println("At Goal " + Timer.getFPGATimestamp());
      end = true;
      return;
    }         

    // ChassisSpeeds = new ChassisSpeeds(null, null, null)
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