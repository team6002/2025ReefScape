package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.Constants.GroundPivotConstants;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_DeployGroundPivot extends Command{
    private boolean deployable;//reasonably confident we can deploy the ground intake
    private boolean isFinished;//variable to keep track of if the pivot is out of the way
    private final SUB_GroundPivot m_groundPivot;
    private final SUB_Pivot m_pivot;
    private final GlobalVariables m_variables;
    public CMD_DeployGroundPivot(SUB_GroundPivot p_groundPivot, SUB_Pivot p_pivot, GlobalVariables p_variables){
        m_groundPivot = p_groundPivot;
        m_pivot = p_pivot;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        deployable = true;
        isFinished = false;//reset so that command is not instantly finished
    }

    @Override
    public void execute(){
        if(m_groundPivot.getPosition() < Math.toRadians(110)){//check to make sure that the ground pivot is actually stowed
            if(m_pivot.getPosition() < Math.toRadians(89)){//check if the pivot is in the way
                // m_pivot.setGoal(Math.toRadians(90));//if ground pivot is stowed and pivot is in the way, move the pivot and not the ground pivot
                deployable = false;//do not finish command yet as pivot is still in a problematic position
            }
        }
        
        //if all checks pass, finish the command and move the ground pivot
        if(deployable){
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        //only move ground pivot if the command was finished naturally
        if(interrupted == false && deployable){
            m_variables.m_groundPivotDeployed = true;
            m_groundPivot.setGoal(GroundPivotConstants.kHome);//when command is finished, deploy the ground pivot
        }
    }
}
