package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class SUB_Arm extends SubsystemBase{
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    public SUB_Arm(ArmIO io){
        this.io = io;

        //tuning
        SmartDashboard.putNumber("armPivotP", ArmConstants.kP);
        SmartDashboard.putNumber("armPivotI", ArmConstants.kI);
        SmartDashboard.putNumber("armPivotD", ArmConstants.kD);
        SmartDashboard.putNumber("armPivotFF", ArmConstants.kFF);
        SmartDashboard.putNumber("armPivotKs", ArmConstants.kS);
        SmartDashboard.putNumber("armPivotKg", ArmConstants.kG);
        SmartDashboard.putNumber("armPivotKv", ArmConstants.kV);
        SmartDashboard.putNumber("armPivotGoal", getPosition());
    }

    public void setGoal(double p_Goal){
      io.setGoal(p_Goal);
    }

    public double getGoal(){
      return inputs.m_armGoal;
    }

    public double getPosition(){
      return inputs.m_armPosition;
    }

    public double getCurrent(){
      return inputs.m_armCurrent;
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Arm", inputs);
      // io.PID();

      //tuning
      // io.setPid(
      //   SmartDashboard.getNumber("armPivotP", 0.0),
      //   SmartDashboard.getNumber("armPivotI", 0.0),
      //   SmartDashboard.getNumber("armPivotD", 0.0),
      //   SmartDashboard.getNumber("armPivotFF", 0.0)
      // );

      // io.setFeedforward(
      //   SmartDashboard.getNumber("armPivotKs", 0.0),
      //   SmartDashboard.getNumber("armPivotKg",0.0),
      //   SmartDashboard.getNumber("armPivotKv", 0.0)
      // );

      // io.setGoal(SmartDashboard.getNumber("armPivotGoal", getPosition()));
    }
}
