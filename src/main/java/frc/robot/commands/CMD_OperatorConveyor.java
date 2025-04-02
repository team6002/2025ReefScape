package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.GlobalVariables;
import frc.robot.subsystems.Intake.SUB_Intake;

public class CMD_OperatorConveyor extends Command{
    private final CommandXboxController m_operatorController;
    private final SUB_Intake m_intake;

    private double lastConveyVoltage;

    public CMD_OperatorConveyor(CommandXboxController p_operatorController, SUB_Intake p_intake, GlobalVariables p_variables){
        m_intake = p_intake;
        m_operatorController = p_operatorController;

        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        lastConveyVoltage = m_intake.getConveyorVoltage();
    }

    @Override
    public void execute(){
        if (DriverStation.isAutonomousEnabled()){this.cancel();return;}
        if(Math.abs(m_operatorController.getLeftY()) > .1){
            m_intake.setConveyorVoltage(-m_operatorController.getLeftY() * 3);
        }else{
            m_intake.setConveyorVoltage(lastConveyVoltage);
        }

        if(!GlobalVariables.m_haveAlgae){
            if(Math.abs(m_operatorController.getRightX()) > .1){
                m_intake.setConveyorVoltage(0);
            }
        }
    }
}
