package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HardwareConstants;

public class ArmIOSparkMax implements ArmIO{
    private final SparkMax m_armMotor;
    private final AbsoluteEncoder m_armEncoder;
    private SparkBaseConfig m_armConfig = new SparkMaxConfig();
    private final SparkClosedLoopController m_armController;

    private double m_armReference;

    public ArmIOSparkMax(){
        //initialize motor
        m_armMotor = new SparkMax(HardwareConstants.kCoralHolderCanId, MotorType.kBrushless);

        //initialize PID controller
        m_armController = m_armMotor.getClosedLoopController();

        //initalize encoder
        m_armEncoder = m_armMotor.getAbsoluteEncoder();

        //setup config
        m_armConfig.inverted(ArmConstants.kArmInverted);
        m_armConfig.idleMode(IdleMode.kBrake);
        m_armConfig.smartCurrentLimit(30);
        m_armConfig.voltageCompensation(12.0);
        m_armConfig.closedLoop.p(ArmConstants.kP);
        m_armConfig.closedLoop.i(ArmConstants.kI);
        m_armConfig.closedLoop.d(ArmConstants.kD);
        m_armConfig.closedLoop.velocityFF(ArmConstants.kFF);
        m_armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_armConfig.closedLoop.outputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

        //apply config
        m_armMotor.configure(m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset reference in init
        m_armReference = 0;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        inputs.m_armReference = getReference();
        inputs.m_armCurrent = getCurrent();
        inputs.m_armPosition = getReference();
    }

    @Override
    public void setReference(double p_reference){
        m_armReference = p_reference;
        m_armController.setReference(m_armReference, ControlType.kVelocity);
    }

    public double getPosition(){
        return m_armEncoder.getPosition();
    }

    public double getCurrent(){
        return m_armMotor.getOutputCurrent();
    }

    public double getReference(){
        return m_armReference;
    }
}
