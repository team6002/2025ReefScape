package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.GlobalVariables;
import frc.robot.subsystems.Intake.SUB_Intake;

public class CMD_OperatorConveyor extends Command{
    private final CommandXboxController m_operatorController;
    private final SUB_Intake m_intake;

    private double lastConveyVoltage, lastIntakeVoltage;

    public CMD_OperatorConveyor(CommandXboxController p_operatorController, SUB_Intake p_intake, GlobalVariables p_variables){
        m_intake = p_intake;
        m_operatorController = p_operatorController;

        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        lastConveyVoltage = m_intake.getConveyorVoltage();
        // lastIntakeVoltage = m_intake.getVoltage();
    }

    @Override
    public void execute(){
        if(Math.abs(m_operatorController.getLeftY()) > .1){
            m_intake.setConveyorVoltage(-m_operatorController.getLeftY() * 3);
            // m_intake.setVoltage(IntakeConstants.kOff);
        }else{
            m_intake.setConveyorVoltage(lastConveyVoltage);
            // if(m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
            //     m_intake.setVoltage(IntakeConstants.kHolding);
            // }else{
            //     m_intake.setVoltage(IntakeConstants.kOff);
            // }
            // m_intake.setVoltage(lastIntakeVoltage);
        }

        if(!GlobalVariables.m_haveAlgae){
            if(Math.abs(m_operatorController.getRightX()) > .1){
                m_intake.setConveyorVoltage(0);
                // m_intake.setVoltage(-m_operatorController.getRightX() * 3);
            }else{
                // if(m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
                //     m_intake.setVoltage(IntakeConstants.kHolding);
                // }else{
                //     m_intake.setVoltage(IntakeConstants.kOff);
                // }
                // m_intake.setVoltage(lastIntakeVoltage);
            }
        }
    }
}
