package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIoInputs {
    public double m_elevatorPos;
    public double m_elevatorGoal;
    public double m_rightElevatorCurrent;
    public double m_leftElevatorCurrent;
    public double m_elevatorSetpoint;
    public double m_rightVoltage;
    public double m_leftVoltage;
    public double m_speed;
    public boolean m_elevatorInPosition;
  }

  public default void updateInputs(ElevatorIoInputs inputs) {}

  public default void setGoal(double p_reference){}

  public default void setVoltage(double voltage){}
  
  public default double getGoal(){return 0;}

  public default double getPosition(){return 0;}

  public default double getCurrent(){return 0;}

  public default double getSetpoint(){return 0;}

  public default boolean inPosition(){return false;}

  public default void PID(){}

  public default void resetEncoder(){}

  public default boolean isResetMode(){return false;}


  public default double getRightVoltage(){return 0;}

  public default double getLeftVoltage(){return 0;}

  public default void reset(boolean m_reset){}
}
