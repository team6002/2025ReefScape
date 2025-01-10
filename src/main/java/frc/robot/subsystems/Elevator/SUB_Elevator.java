package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIoInputsAutoLogged inputs = new ElevatorIoInputsAutoLogged();
    public SUB_Elevator(ElevatorIO io){
        this.io = io;
    }

    public double getPosition(){
      return inputs.m_liftPos;
    }

    public double getCurrent(){
      return inputs.m_liftCurrent;
    }

    public double getGoal(){
     return inputs.m_liftGoal;
    }

    public void setGoal(double p_goal){
      io.setGoal(p_goal);
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Elevator", inputs);
    }
}
