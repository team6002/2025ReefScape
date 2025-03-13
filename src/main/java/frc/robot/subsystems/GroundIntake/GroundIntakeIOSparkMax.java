package frc.robot.subsystems.GroundIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.HardwareConstants;

public class GroundIntakeIOSparkMax implements GroundIntakeIO{
    private final SparkMax m_groundIntakeMotor;
    private final RelativeEncoder m_groundIntakeEncoder;
    private final SparkClosedLoopController m_groundIntakeController;

    private double m_groundIntakeReference;

    public GroundIntakeIOSparkMax(){
        //initialize motor
        m_groundIntakeMotor = new SparkMax(HardwareConstants.kGroundIntakeCanId, MotorType.kBrushless);

        //initialize PID controller
        m_groundIntakeController = m_groundIntakeMotor.getClosedLoopController();

        //initalize encoder
        m_groundIntakeEncoder = m_groundIntakeMotor.getEncoder();

        //apply config
        m_groundIntakeMotor.configure(Configs.GroundIntake.m_groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset target speed in init
        m_groundIntakeReference = 0;
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs){
        inputs.m_groundIntakeReference = getReference();
        inputs.m_groundIntakeCurrent = getCurrent();
        inputs.m_groundIntakeVelocity = getVelocity();
    }

    @Override
    public double getVelocity(){
        return m_groundIntakeEncoder.getVelocity();
    }

    @Override
    public double getCurrent(){
        return m_groundIntakeMotor.getOutputCurrent();
    }

    @Override
    public double getReference(){
        return m_groundIntakeReference;
    }

    @Override
    public void setVoltage(double p_voltage){
        m_groundIntakeReference = p_voltage;
    }

    @Override
    public void PID(){
        m_groundIntakeController.setReference(m_groundIntakeReference, ControlType.kVoltage);
    }
}
