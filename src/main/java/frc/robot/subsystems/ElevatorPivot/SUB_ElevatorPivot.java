package frc.robot.subsystems.ElevatorPivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPivotConstants;

public class SUB_ElevatorPivot extends SubsystemBase{
    private final ElevatorPivotIO io;
    private final ElevatorPivotIoInputsAutoLogged inputs = new ElevatorPivotIoInputsAutoLogged();
    public SUB_ElevatorPivot(ElevatorPivotIO io){
        this.io = io;

        //tuning
        SmartDashboard.putNumber("elevatorPivotP", ElevatorPivotConstants.kP);
        SmartDashboard.putNumber("elevatorPivotI", ElevatorPivotConstants.kI);
        SmartDashboard.putNumber("elevatorPivotD", ElevatorPivotConstants.kD);
        SmartDashboard.putNumber("elevatorPivotFF", ElevatorPivotConstants.kFF);
        SmartDashboard.putNumber("elevatorPivotKs", ElevatorPivotConstants.kS);
        SmartDashboard.putNumber("elevatorPivotKg", ElevatorPivotConstants.kG);
        SmartDashboard.putNumber("elevatorPivotKv", ElevatorPivotConstants.kV);
        SmartDashboard.putNumber("elevatorPivotGoal", getPosition());
        SmartDashboard.putNumber("elevatorPivotPos", getPosition());
        SmartDashboard.putNumber("elevatorPivotCurrent", getCurrent());
    }

    public double getPosition(){
      return inputs.m_pivotPos;
    }

    public double getCurrent(){
      return inputs.m_pivotCurrent;
    }

    public double getGoal(){
     return inputs.m_pivotGoal;
    }

    public void setGoal(double p_goal){
      io.setGoal(p_goal);
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("ElevatorPivot", inputs);
      io.PID();
      io.setSpeed(0.001);
      //tuning
      // io.setPid(
      //   SmartDashboard.getNumber("elevatorPivotP", 0.0),
      //   SmartDashboard.getNumber("elevatorPivotI", 0.0),
      //   SmartDashboard.getNumber("elevatorPivotD", 0.0),
      //   SmartDashboard.getNumber("elevatorPivotFF", 0.0)
      // );
      
      // io.setFeedforward(
      //   SmartDashboard.getNumber("elevatorPivotKs", 0.0),
      //   SmartDashboard.getNumber("elevatorPivotKg",0.0),
      //   SmartDashboard.getNumber("elevatorPivotKv", 0.0)
      // );
      
      SmartDashboard.putNumber("elevatorPivotPos", getPosition());
      SmartDashboard.putNumber("elevatorPivotCurrent", getCurrent());

      io.setGoal(SmartDashboard.getNumber("elevatorPivotGoal", getGoal()));
    }
}