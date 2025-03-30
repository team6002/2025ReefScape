package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.SUB_Intake;

public class CMD_OperatorConveyor extends Command{
    private final CommandXboxController m_operatorController;
    private final SUB_Intake m_intake;
    private final GlobalVariables m_variables;
    public CMD_OperatorConveyor(CommandXboxController p_operatorController, SUB_Intake p_intake, GlobalVariables p_variables){
        m_intake = p_intake;
        m_variables = p_variables;
        m_operatorController = p_operatorController;

        addRequirements(m_intake);
    }

    @Override
    public void execute(){
        if(m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
            //if ready to deploy and left Y is greater than deadzone, set voltage based on input, otherwise turn off
            if(m_operatorController.getLeftY() > .1){
                m_intake.setConveyorVoltage(-m_operatorController.getLeftY() * 3);
            }else{
                m_intake.setConveyorVoltage(0);
            }
            //same as ready to deploy except keep conveyor on if controller is less than deadzone
        }else if(m_variables.isRobotState(RobotState.DEPLOY)){
            if(m_operatorController.getLeftY() > .1){
                m_intake.setConveyorVoltage(-m_operatorController.getLeftY() * 3);
            }else{
                m_intake.setConveyorVoltage(IntakeConstants.kConveyorDeploy);
            }
        }
    }
}
