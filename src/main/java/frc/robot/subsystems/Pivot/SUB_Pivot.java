package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.GlobalVariables;

public class SUB_Pivot extends SubsystemBase{
    private final PivotIO io;
    private final PivotIoInputsAutoLogged inputs = new PivotIoInputsAutoLogged();
    public SUB_Pivot(PivotIO io){
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

    public void reset(){
      io.reset();
    }

    public double getSetpoint(){
      return io.getSetpoint();
    }

    public boolean inPosition(){
      return io.inPosition();
    }

    public Command incrementGoal(double increment){
      return Commands.runOnce(() -> io.setGoal(increment + getGoal()));
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("ElevatorPivot", inputs);
      io.PID();
      // SmartDashboard.putNumber("elevatorPivotPos", Math.toDegrees(getPosition()));
      // SmartDashboard.putNumber("elevatorPivotCurrent", getCurrent());
      // SmartDashboard.putNumber("elevatorPivotGoal", Math.toDegrees(getGoal()));
      // SmartDashboard.putNumber("pivot setpoint", Math.toDegrees(getSetpoint()));
      GlobalVariables.m_pivotAngle = getPosition();
    }
}