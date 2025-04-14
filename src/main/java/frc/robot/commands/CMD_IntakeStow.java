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
        // isFinished = false;
        m_intakeTimer.reset();
        m_intakeTimer.stop();
    }

    @Override
    public void execute(){

            // isFinished = true;
        if (m_intake.hasCoral()){
            m_intakeTimer.start();
        }
    }

    @Override
    public void end(boolean interrupted){
        m_intake.setVoltage(IntakeConstants.kHolding);
    }

    @Override
    public boolean isFinished(){
        return m_intake.hasCoral();
    }
}
