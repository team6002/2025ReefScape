package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;

public class CMD_IntakeJackhammer extends Command{
    private final SUB_CoralHolder m_intake;
    private final Timer m_timer = new Timer();
    private boolean outake = true;
    public CMD_IntakeJackhammer(SUB_CoralHolder p_intake){
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
        if(m_timer.hasElapsed(.025)){
            m_timer.reset();
            if(outake){
                m_intake.setVoltage(-12);
            }else{
                m_intake.setVoltage(0);
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
        m_intake.setVoltage(0);
    }
}
