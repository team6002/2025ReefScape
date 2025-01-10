package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIoInputs {
    public double m_liftPos;
    public double m_liftGoal;
    public double m_liftCurrent;
  }

  public default void updateInputs(ElevatorIoInputs inputs) {}

  public default void setGoal(){}

  public default void updateInputs(ElevatorIoInputs inputs) {}

  public default void setGoal(){}

  public default void getGoal(){}
}
