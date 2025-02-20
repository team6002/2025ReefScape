package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.SUB_Drivetrain;

public class CMD_DriveDigital extends Command {

  private final SUB_Drivetrain m_drivetrain;
  private final CommandXboxController m_controller;

  double deadzone = 0.1;	//variable for amount of deadzone
  double y = 0;           //variable for forward/backward movement
  double x = 0;           //variable for side to side movement
  double rot = 0;        //variable for turning mo vement
  double sideMod = 1; // variable for which side is the robot on
  double seconds;
  boolean right;
  Timer timer = new Timer();
  boolean m_autoSlew;
  public CMD_DriveDigital(SUB_Drivetrain p_drivetrain, CommandXboxController p_controller, boolean right, double seconds) {
    m_drivetrain = p_drivetrain;
    m_controller = p_controller;
    this.seconds = seconds;
    this.right = right;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    if (right){
      y = .3;
    }else{
      y = -.3;
    }
    timer.start();
  }
 
  @Override
  public void execute() {

    // System.out.println(m_drivetrain.autoAlignTurn(m_drivetrain.calculateTargetAngle()));
    m_drivetrain.drive( 0, y, 0,false);
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
      timer.reset();
      timer.stop();
      m_drivetrain.drive(0.0, 0.0, 0.0, true);
  }

  @Override
    public boolean isFinished(){
        return timer.advanceIfElapsed(.15);
    }

}