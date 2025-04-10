package frc.robot.subsystems.GroundIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
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
    private final SparkLimitSwitch m_leftLimit;
    private final SparkLimitSwitch m_rightLimit;

    private double m_groundIntakeReference;
    private ControlType m_groundIntakeType;

    public GroundIntakeIOSparkMax(){
        //initialize motor
        m_groundIntakeMotor = new SparkMax(HardwareConstants.kGroundIntakeCanId, MotorType.kBrushless);

        //initialize PID controller
        m_groundIntakeController = m_groundIntakeMotor.getClosedLoopController();

        //initalize encoder
        m_groundIntakeEncoder = m_groundIntakeMotor.getEncoder();

        //apply config
        m_groundIntakeMotor.configure(Configs.GroundIntake.m_groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_leftLimit = m_groundIntakeMotor.getForwardLimitSwitch();
        m_rightLimit = m_groundIntakeMotor.getReverseLimitSwitch();
        //reset target speed in init
        m_groundIntakeReference = 0;
        m_groundIntakeType = ControlType.kVoltage;
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs){
        inputs.m_groundIntakeReference = getReference();
        inputs.m_groundIntakeCurrent = getCurrent();
        inputs.m_groundIntakeVelocity = getVelocity();
        inputs.m_leftPressed = hasLeftCoral();
        inputs.m_rightPressed = hasRightCoral();
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
        m_groundIntakeType = ControlType.kVoltage;
    }

    @Override
    public void setReference(double p_velocity){
        m_groundIntakeReference = p_velocity;
        m_groundIntakeType = ControlType.kVelocity;
    }


    @Override
    public boolean hasLeftCoral(){
        return m_leftLimit.isPressed();
    }

    @Override
    public boolean hasRightCoral(){
        return m_rightLimit.isPressed();
    }

    @Override
    public void PID(){
        m_groundIntakeController.setReference(m_groundIntakeReference, m_groundIntakeType);
    }
}
