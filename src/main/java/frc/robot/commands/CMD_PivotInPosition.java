package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_PivotInPosition extends Command{
    SUB_ElevatorPivot m_pivot;
    public CMD_PivotInPosition(SUB_ElevatorPivot p_pivot){
        m_pivot = p_pivot;
    }

    @Override
    public boolean isFinished(){
        return m_pivot.inPosition();
    }
}
