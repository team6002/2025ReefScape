package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;

public class CMD_IntakeStow extends Command{
    SUB_CoralHolder m_intake;
    boolean isFinished = false;
    Timer m_intakeTimer = new Timer();
    public CMD_IntakeStow(SUB_CoralHolder p_intake){
        m_intake = p_intake;
    }

    @Override
    public void initialize(){
        isFinished = false;
    }

    @Override
    public void execute(){
        if(m_intake.getCurrent() > 20){
            m_intakeTimer.start();
        }else{
            m_intakeTimer.reset();
        }

        if(m_intakeTimer.get() > 0.1){
            isFinished = true;
        }
    }


    @Override 
    public void end(boolean interrupted){
        m_intake.setVoltage(CoralHolderConstants.kHolding);
    }

    @Override
    public boolean isFinished(){

        return isFinished;
    }
}
