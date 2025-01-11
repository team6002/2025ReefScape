package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIoInputs {
    public double m_liftPos;
    public double m_liftGoal;
    public double m_liftCurrent;
    public double m_pivotPos;
    public double m_pivotGoal;
    public double m_pivotCurrent;
  }

  public default void updateInputs(ElevatorIoInputs inputs) {}

  public default void setLiftGoal(double p_reference){}

  public default void setPivotGoal(double p_reference){}

  public default void getGoal(){}
}
