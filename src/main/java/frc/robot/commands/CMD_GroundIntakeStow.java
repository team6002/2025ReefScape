package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;

public class CMD_GroundIntakeStow extends Command{
    private final SUB_GroundIntake m_groundIntake;
    private boolean isFinished = false;
    private final Timer m_groundIntakeTimer = new Timer();
    public CMD_GroundIntakeStow(SUB_GroundIntake p_groundIntake){
        m_groundIntake = p_groundIntake;
        addRequirements(m_groundIntake);
    }

    @Override
    public void initialize(){
        isFinished = false;
    }

    @Override
    public void execute(){
        if(m_groundIntake.getCurrent() > 12){
            m_groundIntakeTimer.start();
        }else{
            m_groundIntakeTimer.reset();
        }

        if(m_groundIntakeTimer.get() > 0.1){
            isFinished = true;
        }
    }


    @Override 
    public void end(boolean interrupted){
        m_groundIntake.setVoltage(CoralHolderConstants.kHolding);
    }

    @Override
    public boolean isFinished(){

        return isFinished;
    }
}
