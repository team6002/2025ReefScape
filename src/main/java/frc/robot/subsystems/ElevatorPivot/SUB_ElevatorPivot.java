package frc.robot.subsystems.ElevatorPivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_ElevatorPivot extends SubsystemBase{
    private final ElevatorPivotIO io;
    private final ElevatorPivotIoInputsAutoLogged inputs = new ElevatorPivotIoInputsAutoLogged();
    public SUB_ElevatorPivot(ElevatorPivotIO io){
        this.io = io;
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

    public Command incrementGoal(double increment){
      return Commands.runOnce(() -> io.setGoal(increment + getGoal()));
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("ElevatorPivot", inputs);
      io.PID();
 
      SmartDashboard.putNumber("elevatorPivotPos", getPosition());
      SmartDashboard.putNumber("elevatorPivotCurrent", getCurrent());
      SmartDashboard.putNumber("elevatorPivotGoal", getGoal());
    }
}