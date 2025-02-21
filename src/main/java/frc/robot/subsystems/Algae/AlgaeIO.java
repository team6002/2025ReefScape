package frc.robot.subsystems.Algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
    @AutoLog
    public static class AlgaeIoInputs{
        public double m_algaeCurrent;
        public double m_algaeVelocity;
        public double m_algaeReference;
    }

    public default void updateInputs(AlgaeIoInputs inputs){}

    public default void setReference(double p_voltage){}

    public default double getReference(){return 0;}

    public default double getVelocity(){return 0;}

    public default double getCurrent(){return 0;}

    public default void PID(){}
}
