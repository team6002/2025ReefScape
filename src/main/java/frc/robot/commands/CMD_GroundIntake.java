package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.GroundPivotConstants;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;

public class CMD_GroundIntake extends Command{
    private final SUB_GroundPivot m_groundPivot;
    private final SUB_GroundIntake m_groundIntake;
    private final CommandXboxController m_driverController;
    private boolean m_haveCoral = false;
    public CMD_GroundIntake(SUB_GroundPivot p_groundPivot, SUB_GroundIntake p_groundIntake, CommandXboxController p_driverController){
        m_groundPivot = p_groundPivot;
        m_groundIntake = p_groundIntake;
        m_driverController = p_driverController;
        addRequirements(m_groundPivot, m_groundIntake);
    }

    public void initialize(){
        SequentialCommandGroup command;
        if(m_driverController.leftBumper().getAsBoolean() == false && m_groundPivot.getGoal() > Math.toRadians(180)){
            command = new SequentialCommandGroup(
                new InstantCommand(()-> m_groundPivot.setGoal(GroundPivotConstants.kHome))
                ,new InstantCommand(()-> m_groundIntake.setVoltage(GroundIntakeConstants.kOff))
                ,new InstantCommand(()-> m_haveCoral = false)
            );
            command.schedule();
        }else{
            if(m_haveCoral == false){
                command = new SequentialCommandGroup(
                    new InstantCommand(()-> m_groundPivot.setGoal(GroundPivotConstants.kIntake))
                    ,new InstantCommand(()-> m_groundIntake.setVoltage(GroundIntakeConstants.kIntake))
                    ,new CMD_GroundIntakeStow(m_groundIntake)
                    ,new InstantCommand(()-> m_groundPivot.setGoal(GroundPivotConstants.kHome))
                    ,new InstantCommand(()-> m_groundIntake.setVoltage(GroundIntakeConstants.kHolding))
                    ,new InstantCommand(()-> m_haveCoral = true)
                );
                command.addRequirements(m_groundPivot, m_groundIntake);
                command.schedule();
            }else{
                command = new SequentialCommandGroup(
                    new InstantCommand(()-> m_groundPivot.setGoal(GroundPivotConstants.kDeploy))
                    ,new WaitCommand(.2)
                    ,new CMD_GroundPivotInPosition(m_groundPivot).withTimeout(.5)
                    ,new InstantCommand(()-> m_groundIntake.setVoltage(GroundIntakeConstants.kReverse))
                    ,new WaitCommand(.33)
                    ,new InstantCommand(()-> m_groundIntake.setVoltage(GroundIntakeConstants.kOff))
                    ,new InstantCommand(()-> m_groundPivot.setGoal(GroundPivotConstants.kHome))
                    ,new InstantCommand(()-> m_haveCoral = false)
                );
                command.addRequirements(m_groundPivot, m_groundIntake);
                command.schedule();
            }
        }
    }

    public boolean isFinished(){
        return true;
    }
}