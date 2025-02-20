package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.SUB_Elevator;

public class CMD_ElevatorBelowPosition extends Command{
    SUB_Elevator m_elevator;
    double goal;
    boolean inverted;
    public CMD_ElevatorBelowPosition(SUB_Elevator p_elevator, double goal, boolean inverted){
        m_elevator = p_elevator;
        this.inverted = inverted;
        this.goal = goal;
    }

    @Override
    public boolean isFinished(){
        return m_elevator.BelowPosition(goal, inverted);
    }
}
