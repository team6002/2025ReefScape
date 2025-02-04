package frc.robot.subsystems.ElevatorPivot;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorPivotIO {
  @AutoLog
  public static class ElevatorPivotIoInputs {
    public double m_pivotPos;
    public double m_pivotGoal;
    public double m_pivotCurrent;
  }

  public default void updateInputs(ElevatorPivotIoInputs inputs) {}

  public default void setGoal(double p_reference){}

  public default double getGoal(){return 0;}

  public default double getPosition(){return 0;}

  public default double getCurrent(){return 0;}

  public default double getSetpoint(){return 0;}
  
  public default void PID(){}

  public default void reset(){}
}
