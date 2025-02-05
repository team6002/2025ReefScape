package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_PivotInPosition extends Command{
    SUB_ElevatorPivot m_elevatorPivot;
    public CMD_PivotInPosition(SUB_ElevatorPivot p_elevatorPivot){
        m_elevatorPivot = p_elevatorPivot;
    }

    @Override
    public boolean isFinished(){
        return Math.abs(m_elevatorPivot.getGoal() - m_elevatorPivot.getPosition()) < Math.toRadians(1);
    }
}
