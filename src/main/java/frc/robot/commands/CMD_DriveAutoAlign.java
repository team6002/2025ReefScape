package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.Vision.SUB_Vision;

public class CMD_DriveAutoAlign extends Command {

  private final SUB_Drivetrain m_drivetrain;
  private final CommandXboxController m_controller;
  private final SUB_Vision m_vision;

  double deadzone = 0.1;	//variable for amount of deadzone
  double y = 0;           //variable for forward/backward movement
  double x = 0;           //variable for side to side movement
  double rot = 0;        //variable for turning mo vement
  double sideMod = 1; // variable for which side is the robot on
  double xAdjustment=0;
  double yAdjustment=0;
  double rotAdjustment=0;
  double xGoal = 0;
  double yGoal = 5;
  double rotGoal = 0;
  Pose2d goalPose;
  Translation2d FieldCentricTranslation = new Translation2d(0.0, 0.0);//puts the controles back to field centric mode 
  Translation2d adjustedAdjustments = new Translation2d(0.0, 0.0);
  
  public CMD_DriveAutoAlign(SUB_Drivetrain p_drivetrain, CommandXboxController p_controller, SUB_Vision p_vision, double yGoal) {
    m_drivetrain = p_drivetrain;
    m_controller = p_controller;
    m_vision = p_vision;
    addRequirements(m_drivetrain);
    this.yGoal = yGoal;
  }
 
  @Override
  public void execute() {
    try {
      //check if we have vision 
      if (m_vision.getHasLTarget() || m_vision.getHasRTarget()){
        if (Math.abs((m_drivetrain.getTargetOdo().getX() - Units.inchesToMeters(xGoal))) <= Units.inchesToMeters(.5)){
          xAdjustment = 0;
        }
        
        if (Math.abs((Units.inchesToMeters(yGoal) - m_drivetrain.getTargetOdo().getY())) <= Units.inchesToMeters(1)){
          yAdjustment = 0;
        }else{
          var direction = Math.signum(Units.inchesToMeters(yGoal)- m_drivetrain.getTargetOdo().getY() );
          
          yAdjustment = .025 * direction;
        
        }

        if (Math.abs(m_drivetrain.getTargetOdo().getRotation().getDegrees()) <= 3){
          rotAdjustment = 0;  
        }else{
          var turnDirection = Math.signum(m_drivetrain.getTargetOdo().getRotation().getDegrees());
          rotAdjustment = -.005 * turnDirection;
        }    
      }else{
        xAdjustment = 0;
        yAdjustment = 0;
        rotAdjustment = 0;
      }  
    } catch (Exception e) {
    
    }

    //read the controller 
    var ySpeed = MathUtil.applyDeadband(m_controller.getLeftX(),deadzone)*sideMod;

    var xSpeed = MathUtil.applyDeadband(m_controller.getLeftY(),deadzone)*sideMod;

    rot = MathUtil.applyDeadband(-m_controller.getRightX(), deadzone);

    FieldCentricTranslation = new Translation2d(xSpeed, ySpeed).rotateBy(Rotation2d.fromDegrees(m_drivetrain.getAngle())).unaryMinus();

    m_drivetrain.drive(FieldCentricTranslation.getX(), FieldCentricTranslation.getY() + yAdjustment, rot + rotAdjustment, false);
  }

  @Override
  public void end(boolean interrupted) {
      m_drivetrain.drive(0.0, 0.0, 0.0, true);
  }

}