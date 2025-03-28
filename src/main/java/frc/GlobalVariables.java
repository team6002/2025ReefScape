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
    public static boolean m_intakingAlgae = false;
    public static boolean m_groundPivotDeployed = false;
    public static boolean m_intakeFromStation = false;
    public static boolean m_alignBeforeShoot = true;
    public static boolean m_groundHasCoral = false;

    public enum RobotState{
        HOME
        ,READY
        ,READY_TO_INTAKE
        ,READY_TO_DEPLOY
        ,DEPLOY
        ,READY_TO_CLIMB
        ,CLIMB
        ,ALGAE_LEVEL_2
        ,ALGAE_LEVEL_3
        ,PROCESSOR
        ,BARGE
        ,CORAL
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

    @Override
    public void periodic(){
        Logger.recordOutput("GlobalVariables/robotState", getRobotState());
        Logger.recordOutput("GlobalVariables/haveAlgae", m_haveAlgae);
        Logger.recordOutput("GlobalVariables/haveCoral", m_haveCoral);
        Logger.recordOutput("GlobalVariables/targetCoralLevel", m_targetCoralLevel);
        Logger.recordOutput("GlobalVariables/intaking algae", m_intakingAlgae);
    }
}