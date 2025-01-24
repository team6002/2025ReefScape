package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.SUB_Elevator;

public class CMD_ElevatorInPosition extends Command{
    SUB_Elevator m_elevator;
    public CMD_ElevatorInPosition(SUB_Elevator p_elevator){
        m_elevator = p_elevator;
    }

    @Override
    public boolean isFinished(){
        return Math.abs(m_elevator.getElevatorGoal() - m_elevator.getElevatorPosition()) < .1;
    }
}
