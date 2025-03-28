package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;

public class CMD_Deploy extends Command{
    private final SUB_Intake m_intake;
    private final SUB_FlippyWrist m_flippyWrist;
    private final GlobalVariables m_variables;

    public CMD_Deploy(SUB_Intake p_intake, SUB_FlippyWrist p_flippyWrist, GlobalVariables p_variables){
        m_flippyWrist = p_flippyWrist;
        m_intake = p_intake;
        m_variables = p_variables;

        addRequirements(m_intake, m_flippyWrist);
    }

    @Override
    public void initialize(){
        m_intake.setVoltage(IntakeConstants.kReverse);
        if(GlobalVariables.m_targetCoralLevel == 4){
            m_flippyWrist.setGoal(FlippyWristConstants.kStowing);
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
