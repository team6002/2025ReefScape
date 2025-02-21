package frc.robot.subsystems.Algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.HardwareConstants;

public class AlgaeIOSparkMax implements AlgaeIO{
    private final SparkMax m_algaeMotor = new SparkMax(HardwareConstants.kAlgaeCanId, MotorType.kBrushless);
    private final SparkClosedLoopController m_algaeController;
    private final RelativeEncoder m_algaeEncoder;
    private double m_algaeReference = 0;
    public AlgaeIOSparkMax(){
        m_algaeMotor.configure(Configs.AlgaeConfig.m_AlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_algaeController = m_algaeMotor.getClosedLoopController();

        m_algaeEncoder = m_algaeMotor.getEncoder();
    }

    @Override
    public void updateInputs(AlgaeIoInputs inputs){
        inputs.m_algaeCurrent = getCurrent();
        inputs.m_algaeVelocity = getVelocity();
        inputs.m_algaeReference = getReference();
    }

    @Override
    public void setReference(double p_voltage) {
        m_algaeReference = p_voltage;
    }

    @Override
    public double getReference(){
        return m_algaeReference;
    }

    @Override
    public double getVelocity(){
        return m_algaeEncoder.getVelocity();
    }

    @Override
    public double getCurrent(){
        return m_algaeMotor.getAppliedOutput();
    }
    
    @Override
    public void PID(){
        m_algaeController.setReference(m_algaeReference, ControlType.kVoltage);
    }
}
