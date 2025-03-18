package frc;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GlobalVariables extends SubsystemBase{
    public GlobalVariables(){}

    public static int m_targetCoralLevel = 4;

    public static double m_pivotAngle = 0;
    public static double m_elevatorExtension = 0;
    public static boolean m_haveAlgae = false;
    public static boolean m_haveCoral = false;
    public static boolean m_coralException = false;
    public static boolean m_algaeExceptionMode = false;
    public static boolean lvl3AlgaeException = false;
    public static boolean m_intakingAlgae = false;
    public static boolean m_groundPivotDeployed = false;

    public enum RobotState{
        HOME
        ,TRANSITIONING_TO_INTAKE
        ,READY_TO_INTAKE
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

    public enum AlgaeTarget{
        LEVEL_2
        ,LEVEL_3
        ,PROCESSOR
        ,BARGE
        ,GROUND
        ,CORAL
    }

    AlgaeTarget m_algaeTarget = AlgaeTarget.LEVEL_2;

    public AlgaeTarget getAlgaeTarget(){
        return m_algaeTarget;
    }

    public void setAlgaeTarget(AlgaeTarget p_algaeTarget){
        m_algaeTarget = p_algaeTarget;
    }

    @Override
    public void periodic(){
        Logger.recordOutput("GlobalVariables/robotState", getRobotState());
        Logger.recordOutput("GlobalVariables/algaeTarget", getAlgaeTarget());
        Logger.recordOutput("GlobalVariables/haveAlgae", m_haveAlgae);
        Logger.recordOutput("GlobalVariables/haveCoral", m_haveCoral);
        Logger.recordOutput("GlobalVariables/targetCoralLevel", m_targetCoralLevel);
        Logger.recordOutput("GlobalVariables/exceptionMode", m_algaeExceptionMode);
        Logger.recordOutput("GlobalVariables/Lvl3AlgaeExcept", lvl3AlgaeException);
        Logger.recordOutput("GlobalVariables/intaking algae", m_intakingAlgae);
    }
}