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
    private final SparkMax m_coralHolderMotor;
    private final RelativeEncoder m_coralHolderEncoder;
    private final SparkClosedLoopController m_coralHolderController;
    private SimpleMotorFeedforward m_coralHolderFeedforward = new SimpleMotorFeedforward(CoralHolderConstants.kS, CoralHolderConstants.kV);

    private double m_coralHolderReference;

    public CoralHolderIOSparkMax(){
        //initialize motor
        m_coralHolderMotor = new SparkMax(HardwareConstants.kCoralHolderCanId, MotorType.kBrushless);

        //initialize PID controller
        m_coralHolderController = m_coralHolderMotor.getClosedLoopController();

        //initalize encoder
        m_coralHolderEncoder = m_coralHolderMotor.getEncoder();

        //apply config
        m_coralHolderMotor.configure(Configs.CoralHolderConfig.m_coralHolderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        m_coralHolderFeedforward = new SimpleMotorFeedforward(CoralHolderConstants.kS, CoralHolderConstants.kV);
        if(p_rpm < 0){
            Configs.CoralHolderConfig.m_coralHolderConfig.closedLoop.p(CoralHolderConstants.kPReverse);
        }else{
            Configs.CoralHolderConfig.m_coralHolderConfig.closedLoop.p(CoralHolderConstants.kP);
        }
        m_coralHolderMotor.configure(Configs.CoralHolderConfig.m_coralHolderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_coralHolderReference = p_rpm;
    }

    @Override
    public void setVoltage(double p_voltage){
        m_coralHolderFeedforward = new SimpleMotorFeedforward(0, 0);
        m_coralHolderController.setReference(p_voltage, ControlType.kVoltage);
    }

    @Override
    public double getVelocity(){
        return m_coralHolderEncoder.getVelocity();
    }

    @Override
    public double getCurrent(){
        return m_coralHolderMotor.getOutputCurrent();
    }

    @Override
    public double getReference(){
        return m_coralHolderReference;
    }

    @Override
    public void PID(){
        m_coralHolderController.setReference(m_coralHolderReference, ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0, m_coralHolderFeedforward.calculate(getReference()));
    }
}
