package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;

public class CMD_CheckCoral extends Command{
    private final SUB_CoralHolder m_intake;
    private boolean isFinished;
    private Timer m_runTime = new Timer();
    private Timer m_triggerTimer = new Timer();
    public CMD_CheckCoral(SUB_CoralHolder p_intake){
        m_intake = p_intake;
    }

    @Override
    public void initialize(){
        m_runTime.reset();
        m_runTime.start();
        m_triggerTimer.reset();
        isFinished = false;
        m_intake.setVoltage(CoralHolderConstants.kIntake);
    }

    @Override
    public void execute(){
        if(m_intake.getCurrent() > 12){
            m_triggerTimer.start();
        }else{
            m_triggerTimer.reset();
        }

        if(m_triggerTimer.get() > .1){
            m_intake.setVoltage(CoralHolderConstants.kHolding);
            GlobalVariables.m_haveCoral = true;
            isFinished = true;
        }

        if(m_runTime.get() > .25){
            m_intake.setVoltage(CoralHolderConstants.kOff);
            GlobalVariables.m_haveCoral = false;
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
