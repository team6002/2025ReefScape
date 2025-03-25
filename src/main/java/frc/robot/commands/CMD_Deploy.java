package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_Deploy extends Command{
    private final SUB_Intake m_intake;
    private final SUB_Wrist m_wrist;
    private final GlobalVariables m_variables;

    public CMD_Deploy(SUB_Intake p_intake, SUB_Wrist p_wrist, GlobalVariables p_variables){
        m_wrist = p_wrist;
        m_intake = p_intake;
        m_variables = p_variables;

        addRequirements(m_intake, m_wrist);
    }

    @Override
    public void initialize(){
        m_intake.setVoltage(IntakeConstants.kReverse);
        if(GlobalVariables.m_targetCoralLevel == 4){
            m_wrist.setGoal(WristConstants.kStowing);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        m_variables.setRobotState(RobotState.DEPLOY);
        GlobalVariables.m_haveCoral = false;
    }
}
