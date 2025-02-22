package frc;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GlobalVariables extends SubsystemBase{
    public GlobalVariables(){}

    public static int m_targetCoralLevel = 4;

    public static double m_pivotAngle = 0;
    public static double m_elevatorExtension = 0;
    public static boolean m_haveAlgae = false;
    public static boolean m_haveCoral = false;

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
        // SmartDashboard.putString("robotState", getRobotState().toString());
        // SmartDashboard.putString("algae target", getAlgaeTarget().toString());
        // SmartDashboard.putString("mode", getMode().toString());
        // SmartDashboard.putNumber("score level", m_targetCoralLevel);
        // SmartDashboard.putBoolean("has algae", m_haveAlgae);

        Logger.recordOutput("robotState", getRobotState());
        Logger.recordOutput("algaeTarget", getAlgaeTarget());
        Logger.recordOutput("Mode", getMode());
        Logger.recordOutput("haveAlgae", m_haveAlgae);
        Logger.recordOutput("haveCoral", m_haveCoral);
        Logger.recordOutput("targetCoralLevel", m_targetCoralLevel);
    }
}
 