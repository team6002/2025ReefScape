package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
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
  public void initialize() {
    // if (DriverStation.getAlliance().get() == Alliance.Red){
    //   sideMod = 1;
    // }else {
    //   sideMod = -1;
    // }
  }
 
  @Override
  public void execute() {
    try {
      //check if we have vision 
      if (m_vision.getHasLTarget() || m_vision.getHasRTarget()){
        if (Math.abs((m_drivetrain.getTargetOdo().getX() - Units.inchesToMeters(xGoal))) <= Units.inchesToMeters(.5)){
          xAdjustment = 0;
        }else{
          // xAdjustment = -.1;
        
        }
        
        if (Math.abs((Units.inchesToMeters(yGoal) - m_drivetrain.getTargetOdo().getY())) <= Units.inchesToMeters(.5)){
          yAdjustment = 0;
        }else{
          var direction = Math.signum(Units.inchesToMeters(yGoal)- m_drivetrain.getTargetOdo().getY() );
          
          yAdjustment = .025 * direction;
        
        }

        // adjustedAdjustments = new Translation2d(xAdjustment, yAdjustment).rotateBy(Rotation2d.fromDegrees(m_drivetrain.getAngle()));
    
      }  
    } catch (Exception e) {
    
    }

    //read the controller 
    var ySpeed = MathUtil.applyDeadband(m_controller.getLeftX(),deadzone)*sideMod;

    var xSpeed = MathUtil.applyDeadband(m_controller.getLeftY(),deadzone)*sideMod;

    rot = MathUtil.applyDeadband(-m_controller.getRightX(), deadzone);
    // if (m_controller.leftTrigger(.5).getAsBoolean() && m_vision.getTcameraYaw() != Double.MAX_VALUE){
    //   try { 
    //     rot = Math.copySign(.05, -m_vision.getTcameraYaw());
    //   } catch (Exception e) {
    //   } 
    // }
    if (m_vision.getTcameraYaw() !=Double.MAX_VALUE && Math.abs(m_vision.getTcameraYaw()) <= 3.5){
      rot = 0;
    }

    FieldCentricTranslation = new Translation2d(xSpeed, ySpeed).rotateBy(Rotation2d.fromDegrees(m_drivetrain.getAngle()).unaryMinus());
    // System.out.println(m_drivetrain.autoAlignTurn(m_drivetrain.calculateTargetAngle()));
    m_drivetrain.drive(FieldCentricTranslation.getX() + xSpeed, FieldCentricTranslation.getY() + yAdjustment, rot, false);
  }

  // private static double modifyAxis(double value) {
  //   double modifedValue;
  //   // Deadband
  //   // value = deadband(value, 0.2);

  //   // Square the axis
  //   modifedValue = value * value;
  //   modifedValue = Math.copySign(value, value);

  //   return modifedValue;
  // }
  @Override
  public void end(boolean interrupted) {
      m_drivetrain.drive(0.0, 0.0, 0.0, true);
  }

}