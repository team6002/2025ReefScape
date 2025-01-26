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

  public default void setSpeed(double p_speed){}
  
  public default void PID(){}

  public default void setPid(double kP, double kI, double kD, double kFF){}

  public default void setFeedforward(double kS, double kG, double kV){}
}
