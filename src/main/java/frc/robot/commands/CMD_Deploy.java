package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_Deploy extends Command{
    SUB_Wrist m_wrist;
    SUB_CoralHolder m_intake;
    Timer m_runTime = new Timer();
    public CMD_Deploy(SUB_Wrist p_wrist, SUB_CoralHolder p_intake){
        m_wrist = p_wrist;
        m_intake = p_intake;
    }

    @Override
    public void initialize(){ 
        m_runTime.reset();
        m_runTime.start();
        switch (GlobalVariables.m_targetCoralLevel) {
            case 1:
                new CMD_DeployLevelOne(m_intake).schedule();
                break;
            case 2:
                new CMD_DeployLevelTwo(m_intake, m_wrist).schedule();
                break;
            case 3:
                new CMD_DeployLevelThree(m_intake, m_wrist).schedule();
                break;
            case 4:
                new CMD_DeployLevelFour(m_intake, m_wrist).schedule();
                break;
            default:
                break;
        }}

    @Override
    public boolean isFinished(){
        return m_wrist.inPosition() && m_runTime.get() > .5;
    }
}
