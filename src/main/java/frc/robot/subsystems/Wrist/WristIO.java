package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double m_wristCurrent;
    public double m_wristPosition;
    public double m_wristGoal;
    public boolean m_wristInPosition;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setGoal(double p_goal){}

  public default void setGoal(){}
  
  public default void setVoltage(double voltage){}

  public default double getGoal(){return 0;}

  public default double getPosition(){return 0;}

  public default double getCurrent(){return 0;}

  public default double getSetpoint(){return 0;}

  public default boolean inPosition(){return false;}

  public default void PID(){}

  public default void reset(){

  }
}
