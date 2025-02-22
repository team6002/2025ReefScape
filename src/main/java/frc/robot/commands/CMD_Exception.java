package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_Exception extends Command{
    SUB_Wrist m_wrist;
    SUB_Algae m_algae;
    SUB_Elevator m_elevator;
    SUB_Pivot m_pivot;
    SUB_CoralHolder m_intake;
    GlobalVariables m_variables;
    public CMD_Exception(SUB_Wrist p_wrist, SUB_Algae p_algae, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_CoralHolder p_intake,
        GlobalVariables p_variables){

        m_algae = p_algae;
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_intake = p_intake;
        m_wrist = p_wrist;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        if(GlobalVariables.m_haveAlgae && (m_variables.isRobotState(RobotState.READY) || 
            m_variables.isRobotState(RobotState.READY_TO_INTAKE) || m_variables.isRobotState(RobotState.READY_STOWED))){

            //eject algae after intaking off of reef
            new SequentialCommandGroup(
                new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kReverse))
                ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = false)
                ,new WaitCommand(.5)
                ,new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kOff)) 
            ).schedule();
            return;
        }

        if(m_variables.isRobotState(RobotState.READY_TO_INTAKE)){
            //if coral is in the middle of robot & coral chute
            new CMD_ReadyToIntakeException(m_pivot, m_elevator, m_wrist).schedule();
            return;
        }

        if(GlobalVariables.m_haveAlgae && m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
            //eject algae after intaking off of reef
            new CMD_YeetAlgae(m_wrist, m_algae).schedule();
            return;
        }


        if(m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
            new CMD_ReadyToDeployException(m_elevator, m_wrist, m_pivot, m_intake, m_variables).schedule();
            return;
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
