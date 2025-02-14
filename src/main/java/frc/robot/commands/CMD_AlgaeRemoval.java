package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_AlgaeRemoval extends Command{
    private final SUB_Wrist m_wrist;
    private final SUB_ElevatorPivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_Algae m_algae;
    private final GlobalVariables m_variables;
    public CMD_AlgaeRemoval(SUB_Wrist p_wrist, SUB_ElevatorPivot p_pivot, SUB_Elevator p_elevator, SUB_Algae p_algae,
        GlobalVariables p_variales){
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_algae = p_algae;
        m_variables = p_variales;
    }
}
