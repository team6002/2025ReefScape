package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.SUB_Intake;

public class CMD_IntakeStow extends Command{
    SUB_Intake m_intake;
    boolean isFinished = false;
    Timer m_intakeTimer = new Timer();
    public CMD_IntakeStow(SUB_Intake p_intake){
        m_intake = p_intake;
    }

    @Override
    public void initialize(){
        isFinished = false;
    }

    @Override
    public void execute(){
        if(m_intake.getCurrent() > 18){
            m_intakeTimer.start();
        }else{
            m_intakeTimer.reset();
        }

        if(m_intakeTimer.get() > 0.2){
            m_intake.setVoltage(IntakeConstants.kHolding);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished(){

        return isFinished;
    }
}
