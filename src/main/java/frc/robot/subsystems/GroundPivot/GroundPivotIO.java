package frc.robot.subsystems.GroundPivot;

import org.littletonrobotics.junction.AutoLog;

public interface GroundPivotIO {
  @AutoLog
  public static class GroundPivotIOInputs {
    public double m_groundPivotCurrent;
    public double m_groundPivotPosition;
    public double m_groundPivotGoal;
    public boolean m_groundPivotInPosition;
    public double m_groundPivotSetpoint;
  }

  public default void updateInputs(GroundPivotIOInputs inputs) {}

  public default void setGoal(double p_goal){}

  public default void setGoal(){}

  public default void setConstraints(double velocity, double acceleration){};

  public default double getGoal(){return 0;}

  public default double getPosition(){return 0;}

  public default double getCurrent(){return 0;}

  public default double getSetpoint(){return 0;}

  public default boolean inPosition(){return false;}

  public default boolean inPosition(double p_position){return false;}
  

  public default void PID(){}

  public default void reset(){

  }
}
