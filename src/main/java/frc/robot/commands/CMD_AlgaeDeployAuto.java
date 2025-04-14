package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;

public class CMD_AlgaeDeployAuto extends Command{
    private final SUB_Intake m_intake;
    private final SUB_FlippyWrist m_flippyWrist;
    private final GlobalVariables m_variables;

    private final Timer m_timer = new Timer();
    private boolean outake = true;
    private final Timer m_runTime = new Timer();

    public 
    CMD_AlgaeDeployAuto(SUB_Intake p_intake, SUB_FlippyWrist p_flippyWrist, GlobalVariables p_variables){
        m_flippyWrist = p_flippyWrist;
        m_intake = p_intake;
        m_variables = p_variables;

        addRequirements(m_intake, m_flippyWrist);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.stop();

        m_runTime.reset();
        m_runTime.start();

        m_intake.setVoltage(IntakeConstants.kAlgaeDeploy);
    
        // m_variables.setRobotState(RobotState.DEPLOY);
    }

    @Override
    public void execute(){
        if(!m_intake.hasCoral()){
          m_timer.start();
        }
        // m_intake.setConveyorVoltage(IntakeConstants.kConveyorDeploy);
    }

    @Override
    public boolean isFinished(){
        return m_timer.advanceIfElapsed(.2);
    }

    @Override
    public void end(boolean interrupted){
        // GlobalVariables.m_haveCoral = false;
        // m_intake.setConveyorVoltage(0);
        m_intake.setVoltage(0);
    }
}
