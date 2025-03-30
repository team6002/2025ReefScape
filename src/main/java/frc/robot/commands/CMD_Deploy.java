package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;

public class CMD_Deploy extends Command{
    private final SUB_Intake m_intake;
    private final SUB_FlippyWrist m_flippyWrist;
    private final GlobalVariables m_variables;

    private final Timer m_timer = new Timer();
    private boolean outake = true;
    private final Timer m_runTime = new Timer();

    public CMD_Deploy(SUB_Intake p_intake, SUB_FlippyWrist p_flippyWrist, GlobalVariables p_variables){
        m_flippyWrist = p_flippyWrist;
        m_intake = p_intake;
        m_variables = p_variables;

        addRequirements(m_intake, m_flippyWrist);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();

        m_runTime.reset();
        m_runTime.start();
    }

    @Override
    public void execute(){
        if(m_timer.hasElapsed(.1)){
            m_timer.reset();
            if(outake){
                m_intake.setConveyorVoltage(-10);
            }else{
                m_intake.setConveyorVoltage(-2);
            }
            outake = !outake;
        }
    }

    @Override
    public boolean isFinished(){
        return m_runTime.get() > .5;
    }

    @Override
    public void end(boolean interrupted){
        m_variables.setRobotState(RobotState.DEPLOY);
        GlobalVariables.m_haveCoral = false;
        m_intake.setConveyorVoltage(0);
    }
}
