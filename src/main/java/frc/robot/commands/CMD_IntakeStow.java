package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_IntakeStow extends Command{
    SUB_CoralHolder m_intake;
    SUB_Wrist m_wrist;
    SUB_ElevatorPivot m_pivot;
    SUB_Elevator m_elevator;
    SUB_CoralHolder m_coralHolder;
    boolean isFinished = false;
    Timer m_intakeTimer = new Timer();
    public CMD_IntakeStow(SUB_CoralHolder p_intake, SUB_Wrist p_wrist, SUB_ElevatorPivot p_pivot, SUB_Elevator p_elevator, 
                          SUB_CoralHolder p_coralHolder){
        m_intake = p_intake;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_coralHolder = p_coralHolder;
    }

    @Override
    public void initialize(){
        isFinished = false;
    }

    @Override
    public void execute(){
        if(m_intake.getCurrent() > 30){
            m_intakeTimer.start();
        }else{
            m_intakeTimer.reset();
        }

        if(m_intakeTimer.get() > 0.1){
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    // @Override
    // public void end(boolean interrupted){
    //     new CMD_Ready(m_elevator, m_wrist, m_pivot, m_coralHolder, m_variables).schedule();
    // }
}
