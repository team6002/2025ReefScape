package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIoInputs {
    public double m_pivotPos;
    public double m_pivotGoal;
    public double m_pivotCurrent;
    public boolean m_pivotInPosition;
    public double m_pivotSetpoint;
  }

  public default void updateInputs(PivotIoInputs inputs) {}

  public default void setGoal(double p_reference){}

  public default void setVoltage(double voltage){}

  public default double getGoal(){return 0;}

  public default double getPosition(){return 0;}

  public default double getCurrent(){return 0;}

  public default double getSetpoint(){return 0;}

  public default boolean inPosition(){return false;}
  
  public default void PID(){}

  public default void reset(){}
}
