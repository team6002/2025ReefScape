package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_AutoDeploy extends Command{
    private boolean isFinished = false;
    private boolean m_forward = true;
    private final SUB_Wrist m_wrist;
    private final SUB_CoralHolder m_intake;
    private final GlobalVariables m_variables;
    private final CommandXboxController m_driveController;
    private final Timer m_timer = new Timer();
    private final Timer m_sustainTimer = new Timer();
    public CMD_AutoDeploy(SUB_Wrist p_wrist, SUB_CoralHolder p_intake, GlobalVariables p_variables, CommandXboxController p_driveController){
        m_wrist = p_wrist;
        m_intake = p_intake;
        m_variables = p_variables;
        m_driveController = p_driveController;
    }

    @Override
    public void initialize(){
        isFinished = false;
        m_forward = true;
        m_timer.reset();
        m_timer.start();
        m_sustainTimer.reset();
        m_sustainTimer.start();
        m_intake.setReference(-200);
    }

    @Override
    public void execute(){
        if(m_timer.get() > .5){
            if(m_forward == true){
                m_intake.setReference(500);
            }else{
                m_intake.setReference(-200);
            }
            m_forward = !m_forward;
            m_timer.reset();
        }

        if(m_intake.getCurrent() > 20){
            if(m_sustainTimer.get() > .25 && m_forward){
                isFinished = true;
            }
        }else{
            m_sustainTimer.reset();
        }
        m_timer.get();
        m_sustainTimer.get();

        if(m_driveController.rightBumper().getAsBoolean()){
            return;
        }
    }

    @Override
    public boolean isFinished(){

        return isFinished;
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted){
            m_intake.setReference(50);
            return;
        }else{
            m_intake.setReference(CoralHolderConstants.kReverse);
            m_wrist.setGoal(WristConstants.kStowing);
            m_variables.setRobotState(RobotState.DEPLOY);
        }
    }
}
