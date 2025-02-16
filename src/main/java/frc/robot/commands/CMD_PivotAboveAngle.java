package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_PivotAboveAngle extends Command{
    private final SUB_Pivot m_pivot;
    private boolean aboveAngle = false;
    private double m_angle;
    public CMD_PivotAboveAngle(SUB_Pivot p_pivot, double p_angle){
        m_pivot = p_pivot;
        m_angle = p_angle;
    }

    @Override
    public void initialize(){
        aboveAngle = false;
    }

    @Override
    public void execute(){
        aboveAngle = m_pivot.getPosition() > Math.toRadians(m_angle);
    }

    @Override
    public boolean isFinished(){
        return aboveAngle;
    }
}
