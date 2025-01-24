package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.SUB_Elevator;

public class CMD_PivotInPosition extends Command{
    SUB_Elevator m_elevator;
    public CMD_PivotInPosition(SUB_Elevator p_elevator){
        m_elevator = p_elevator;
    }

    @Override
    public boolean isFinished(){
        return Math.abs(m_elevator.getPivotGoal() - m_elevator.getPivotPosition()) < 1;
    }
}
