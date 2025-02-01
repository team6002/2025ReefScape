package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIoInputs {
    public double m_elevatorPos;
    public double m_elevatorGoal;
    public double m_elevatorCurrent;
  }

  public default void updateInputs(ElevatorIoInputs inputs) {}

  public default void setGoal(double p_reference){}

  public default double getGoal(){return 0;}

  public default double getPosition(){return 0;}

  public default double getCurrent(){return 0;}

  public default void PID(){}

  public default void reset(){}
}
