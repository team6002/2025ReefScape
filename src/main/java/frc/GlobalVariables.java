package frc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GlobalVariables extends SubsystemBase{
    public GlobalVariables(){}

    public static int m_targetLevel = 3;

    public static double m_pivotAngle = 0;
    public static double m_elevatorExtension = 0;

    public enum RobotState{
        HOME
        ,TRANSITIONING_TO_INTAKE
        ,TRANSITIONING_TO_READY
        ,READY
        ,READY_TO_INTAKE
        ,TRANSITONING_TO_STOW
        ,STOW
        ,READY_STOWED
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

    public enum Mode{
        OFFENSIVE
        ,DEFENSIVE
    }

    Mode m_mode = Mode.OFFENSIVE;
    
    public void setMode(Mode p_mode){
        m_mode = p_mode;
    }

    public boolean isMode(Mode p_mode){
        return m_mode == p_mode;
    }

    public Mode getMode(){
        return m_mode;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("robotState", getRobotState().toString());
        SmartDashboard.putString("intakeState", getIntakeState().toString());
        SmartDashboard.putString("mode", getMode().toString());
        SmartDashboard.putNumber("score level", m_targetLevel);
    }
}
