package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIoInputsAutoLogged inputs = new ElevatorIoInputsAutoLogged();
    public SUB_Elevator(ElevatorIO io){
        this.io = io;
    }

    public double getElevatorPosition(){
      return inputs.m_ElevatorPos;
    }

    public double getElevatorCurrent(){
      return inputs.m_ElevatorCurrent;
    }

    public double getElevatorGoal(){
     return inputs.m_ElevatorGoal;
    }

    public void setElevatorGoal(double p_goal){
      io.setElevatorGoal(p_goal);
    }

    public double getPivotPosition(){
      return inputs.m_pivotPos;
    }

    public double getPivotCurrent(){
      return inputs.m_pivotCurrent;
    }

    public double getPivotGoal(){
     return inputs.m_pivotGoal;
    }

    public void setPivotGoal(double p_goal){
      io.setPivotGoal(p_goal);
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Elevator", inputs);
    }
}
