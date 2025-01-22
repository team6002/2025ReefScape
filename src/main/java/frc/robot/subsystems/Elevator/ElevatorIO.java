package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIoInputs {
    public double m_ElevatorPos;
    public double m_ElevatorGoal;
    public double m_ElevatorCurrent;
    public double m_pivotPos;
    public double m_pivotGoal;
    public double m_pivotCurrent;
  }

  public default void updateInputs(ElevatorIoInputs inputs) {}

  public default void setElevatorGoal(double p_reference){}

  public default void setPivotGoal(double p_reference){}

  public default void getGoal(){}
}
