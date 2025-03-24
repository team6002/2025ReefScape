package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;

public class CMD_AlageJackhammer extends Command{
    private final SUB_Algae m_intake;
    private final Timer m_timer = new Timer();
    private boolean outake = true;
    public CMD_AlageJackhammer(SUB_Algae p_intake){
        m_intake = p_intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        if(m_timer.hasElapsed(.05)){
            m_timer.reset();
            if(outake){
                m_intake.setReference(0);
            }else{
                m_intake.setReference(AlgaeConstants.kReverse);
            }
            outake = !outake;
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_intake.setReference(0);
    }
}
