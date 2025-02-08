package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.SUB_Elevator;

public class CMD_ElevatorReset extends Command{
    SUB_Elevator m_elevator;
    boolean m_isFinished = false;
    boolean m_isMoving = true;
    Timer m_runTime = new Timer();

    public CMD_ElevatorReset(SUB_Elevator p_elevator) {
        m_elevator = p_elevator;
        addRequirements(p_elevator);
    }

    @Override
    public void initialize() {
        m_isFinished = false;
        m_elevator.reset(true);
        m_isMoving = true;
        m_runTime.reset();
        m_runTime.start();
    }

    @Override
    public void execute(){
        if (m_isMoving) {
            if (m_elevator.getCurrent() > 30) {
                if(m_runTime.get() > .1){
                    m_isMoving = false;
                    m_isFinished = true;
                }
            }else{
                m_runTime.reset();
            }
        }
        
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("done reseting");
        m_elevator.resetEncoder();
    }
}