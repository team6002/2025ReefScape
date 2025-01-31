package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double m_wristCurrent;
    public double m_wristPosition;
    public double m_wristGoal;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void setGoal(double p_goal){}

  public default void setPid(double kP, double kI, double kD, double kFF){}

  public default void setFeedforward(double kS, double kG, double kV){}

  public default void setGoal(){}

  public default double getGoal(){return 0;}

  public default double getPosition(){return 0;}

  public default double getCurrent(){return 0;}

  public default void PID(){}

  public default void reset(){

  }
}
