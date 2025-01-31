package frc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GlobalVariables extends SubsystemBase{
    public GlobalVariables(){}

    public static int m_targetLevel = 3;

    public enum RobotState{
        HOME
        ,TRANSITIONING_TO_INTAKE
        ,READY
        ,READY_TO_INTAKE
        ,TRANSITONING_TO_STOW
        ,STOW
        ,TRANSITIONING_TO_DEPLOY
        ,READY_TO_DEPLOY
        ,DEPLOY
        ,TRANSITIONING_TO_HOME
        ,READY_TO_CLIMB
        ,CLIMB
    }

    RobotState m_robotState = RobotState.HOME;
    
    public void setRobotState(RobotState p_robotState){
        m_robotState = p_robotState;
    }

    public boolean isRobotState(RobotState p_robotState){
        return m_robotState == p_robotState;
    }

    public RobotState getRobotState(){
        return m_robotState;
    }

    public enum IntakeState{
        ALGAE
        ,CORAL
    }

    IntakeState m_intakeState = IntakeState.CORAL;
    
    public void setIntakeState(IntakeState p_intakeState){
        m_intakeState = p_intakeState;
    }

    public boolean isIntakeState(IntakeState p_intakeState){
        return m_intakeState == p_intakeState;
    }

    public IntakeState getIntakeState(){
        return m_intakeState;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("robotState", getRobotState().toString());
        SmartDashboard.putString("intakeState", getIntakeState().toString());
    }
}
