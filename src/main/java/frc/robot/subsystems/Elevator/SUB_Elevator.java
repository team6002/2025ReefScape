package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.GlobalVariables;

public class SUB_Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIoInputsAutoLogged inputs = new ElevatorIoInputsAutoLogged();
    public SUB_Elevator(ElevatorIO io){
        this.io = io;
    }

    public double getPosition(){
      return inputs.m_elevatorPos;
    }

    public double getCurrent(){
      return inputs.m_elevatorCurrent;
    }

    public double getGoal(){
     return inputs.m_elevatorGoal;
    }

    public void setGoal(double p_goal){
      io.setGoal(p_goal);
    }

    public void reset(boolean p_reset){
      io.reset(p_reset);
    }

    public void resetEncoder(){
      io.resetEncoder();
    }

    public double getSetpoint(){
      return io.getSetpoint();
    }

    public boolean inPosition(){
      return io.inPosition();
    }

    public boolean isResetMode(){
      return io.isResetMode();
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Elevator", inputs);
      io.PID();
      // SmartDashboard.putNumber("elevator pos", getPosition());
      // SmartDashboard.putNumber("elevator target", getGoal());
      // SmartDashboard.putNumber("elevator current", getCurrent());
      // SmartDashboard.putNumber("elevator setpoint", getSetpoint());
      // SmartDashboard.putBoolean("resetMode", isResetMode());
      GlobalVariables.m_elevatorExtension = getPosition();
    }
}
