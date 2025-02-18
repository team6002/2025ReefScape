package frc.robot.subsystems.CoralHolder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Configs;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.Constants.HardwareConstants;

public class CoralHolderIOSparkMax implements CoralHolderIO{
    private final SparkMax m_intakeMotor;
    private final RelativeEncoder m_intakeEncoder;
    private final SparkClosedLoopController m_intakeController;
    private SimpleMotorFeedforward m_intakeFeedforward = new SimpleMotorFeedforward(CoralHolderConstants.kS, CoralHolderConstants.kV);

    private double m_intakeReference;

    public CoralHolderIOSparkMax(){
        //initialize motor
        m_intakeMotor = new SparkMax(HardwareConstants.kCoralHolderCanId, MotorType.kBrushless);

        //initialize PID controller
        m_intakeController = m_intakeMotor.getClosedLoopController();

        //initalize encoder
        m_intakeEncoder = m_intakeMotor.getEncoder();

        //apply config
        m_intakeMotor.configure(Configs.CoralHolderConfig.m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset target speed in init
        m_intakeReference = 0;
    }

    @Override
    public void updateInputs(CoralHolderIOInputs inputs){
        inputs.m_intakeReference = getReference();
        inputs.m_intakeCurrent = getCurrent();
        inputs.m_intakeVelocity = getVelocity();
    }

    @Override
    public void setReference(double p_rpm){
        m_intakeFeedforward = new SimpleMotorFeedforward(CoralHolderConstants.kS, CoralHolderConstants.kV);
        if(p_rpm < 0){
            Configs.CoralHolderConfig.m_intakeConfig.closedLoop.p(CoralHolderConstants.kPReverse);
        }else{
            Configs.CoralHolderConfig.m_intakeConfig.closedLoop.p(CoralHolderConstants.kP);
        }
        m_intakeMotor.configure(Configs.CoralHolderConfig.m_intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_intakeReference = p_rpm;
    }

    @Override
    public void setVoltage(double p_voltage){
        m_intakeFeedforward = new SimpleMotorFeedforward(0, 0);
        m_intakeController.setReference(p_voltage, ControlType.kVoltage);
    }

    @Override
    public double getVelocity(){
        return m_intakeEncoder.getVelocity();
    }

    @Override
    public double getCurrent(){
        return m_intakeMotor.getOutputCurrent();
    }

    @Override
    public double getReference(){
        return m_intakeReference;
    }

    @Override
    public void PID(){
        m_intakeController.setReference(m_intakeReference, ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0, m_intakeFeedforward.calculate(getReference()));
    }
}
