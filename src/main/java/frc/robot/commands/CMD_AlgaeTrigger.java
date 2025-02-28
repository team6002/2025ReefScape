package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.subsystems.Algae.SUB_Algae;

public class CMD_AlgaeTrigger extends Command{
    private final SUB_Algae m_algae;
    private final GlobalVariables m_variables;
    private final Timer m_triggerTimer = new Timer();
    private boolean isFinished = false;
    public CMD_AlgaeTrigger(SUB_Algae p_algae, GlobalVariables p_variables){
        m_algae = p_algae;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        m_triggerTimer.reset();
        m_triggerTimer.start();
        isFinished = false;
    }

    @Override
    public void execute(){
        if(m_algae.getCurrent() > 20){
            if(m_triggerTimer.get() > .1){
                isFinished = true;
                m_variables.m_haveAlgae = true;
            }
        }else{
            m_triggerTimer.reset();
        }
    }

    @Override
    public void end(boolean isFinished){
        m_algae.setReference(0);
    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
