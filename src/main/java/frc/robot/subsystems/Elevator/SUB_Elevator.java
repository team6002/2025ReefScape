package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIoInputsAutoLogged inputs = new ElevatorIoInputsAutoLogged();
    public SUB_Elevator(ElevatorIO io){
        this.io = io;
    }

    public double getElevatorPosition(){
      return inputs.m_elevatorPos;
    }

    public double getElevatorCurrent(){
      return inputs.m_elevatorCurrent;
    }

    public double getElevatorGoal(){
     return inputs.m_elevatorGoal;
    }

    public void setGoal(double p_goal){
      io.setGoal(p_goal);
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Elevator", inputs);
      // io.PID();

      SmartDashboard.putNumber("elevator pos", getElevatorPosition());
      SmartDashboard.putNumber("elevator target", getElevatorGoal());
      SmartDashboard.putNumber("elevator current", getElevatorCurrent());
    }
}
