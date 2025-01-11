package frc.robot.subsystems.CoralHolder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CoralHolderConstants;
import frc.robot.Constants.HardwareConstants;

public class CoralHolderIOSparkMax implements CoralIHolderIO{
    private final SparkMax m_coralHolderMotor;
    private final RelativeEncoder m_coralHolderEncoder;
    private SparkBaseConfig m_coralHolderConfig;
    private final SparkClosedLoopController m_coralHolderController;

    private double m_coralHolderReference;

    public CoralHolderIOSparkMax(){
        //initialize motor
        m_coralHolderMotor = new SparkMax(HardwareConstants.kCoralHolderCanId, MotorType.kBrushless);

        //initialize PID controller
        m_coralHolderController = m_coralHolderMotor.getClosedLoopController();

        //initalize encoder
        m_coralHolderEncoder = m_coralHolderMotor.getEncoder();

        //setup config
        m_coralHolderConfig.inverted(CoralHolderConstants.kCoralHolderInverted);
        m_coralHolderConfig.idleMode(IdleMode.kBrake);
        m_coralHolderConfig.smartCurrentLimit(30);
        m_coralHolderConfig.voltageCompensation(12.0);
        m_coralHolderConfig.closedLoop.p(CoralHolderConstants.kCoralHolderP);
        m_coralHolderConfig.closedLoop.i(CoralHolderConstants.kCoralHolderI);
        m_coralHolderConfig.closedLoop.d(CoralHolderConstants.kCoralHolderD);
        m_coralHolderConfig.closedLoop.velocityFF(CoralHolderConstants.kCoralHolderFF);
        m_coralHolderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        m_coralHolderConfig.closedLoop.outputRange(-1, 1);

        //apply config
        m_coralHolderMotor.configure(m_coralHolderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset target speed in init
        m_coralHolderReference = 0;
    }

    @Override
    public void updateInputs(CoralHolderIOInputs inputs){
        inputs.m_intakeReference = getReference();
        inputs.m_intakeCurrent = getCurrent();
        inputs.m_intakeVelocity = getVelocity();
    }

    @Override
    public void setReference(double p_rpm){
        m_coralHolderReference = p_rpm;
        m_coralHolderController.setReference(m_coralHolderReference, ControlType.kVelocity);
    }

    public double getVelocity(){
        return m_coralHolderEncoder.getVelocity();
    }

    public double getCurrent(){
        return m_coralHolderMotor.getOutputCurrent();
    }

    public double getReference(){
        return m_coralHolderReference;
    }
}
