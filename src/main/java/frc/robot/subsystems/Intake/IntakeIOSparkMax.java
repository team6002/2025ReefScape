package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO{
    private final SparkMax m_intakeMotor;
    private final RelativeEncoder m_intakeEncoder;
    private final SparkClosedLoopController m_intakeController;
    private final SparkMax m_conveyorMotor;
    private final SparkClosedLoopController m_conveyorController;

    private double m_intakeReference;
    final SimpleMotorFeedforward m_intakeFeedforward;

    private double m_intakeVoltage;
    private double m_conveyorVoltage;

    public IntakeIOSparkMax(){
        //initialize motor
        m_intakeMotor = new SparkMax(HardwareConstants.kIntakeCanId, MotorType.kBrushless);
        m_conveyorMotor = new SparkMax(HardwareConstants.kIntakeConveyorCanId, MotorType.kBrushless);

        //initialize PID controller
        m_intakeController = m_intakeMotor.getClosedLoopController();
        m_conveyorController = m_conveyorMotor.getClosedLoopController();

        //initalize encoder
        m_intakeEncoder = m_intakeMotor.getEncoder();

        //apply config
        m_intakeMotor.configure(Configs.IntakeConfig.m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_conveyorMotor.configure(Configs.IntakeConfig.m_conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_intakeFeedforward = new SimpleMotorFeedforward(IntakeConstants.kS, 
        IntakeConstants.kV, IntakeConstants.kA);
    
        //reset target speed in init
        m_intakeReference = 0;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.m_intakeReference = getReference();
        inputs.m_intakeCurrent = getCurrent();
        inputs.m_intakeVelocity = getVelocity();
        inputs.m_conveyorCurrent = getConveyorCurrent();
        inputs.m_conveyorVoltage = getConveyorVoltage();
    }

    @Override
    public void setCurrentLimit(int p_limit){
        var IntakeConfig = Configs.IntakeConfig.m_intakeConfig.smartCurrentLimit(p_limit);
        m_intakeMotor.configure(IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getVelocity(){
        return m_intakeEncoder.getVelocity();
    }

    @Override
    public double getVoltage(){
        return m_intakeVoltage;
    }

    @Override
    public double getConveyorVoltage(){
        return m_conveyorVoltage;
    }

    @Override
    public double getCurrent(){
        return m_intakeMotor.getOutputCurrent();
    }

    @Override
    public double getConveyorCurrent(){
        return m_conveyorMotor.getOutputCurrent();
    }

    @Override
    public double getReference(){
        return m_intakeReference;
    }

    @Override
    public void setVoltage(double p_voltage){
        m_intakeVoltage = p_voltage;
        m_intakeController.setReference(p_voltage, ControlType.kVoltage);
    }

    @Override
    public void setReference(double p_speed){
        m_intakeController.setReference(p_speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, m_intakeFeedforward.calculate(p_speed));
    }

    @Override
    public void setConveyorVoltage(double p_voltage){
        m_conveyorVoltage = p_voltage;
        m_conveyorController.setReference(p_voltage, ControlType.kVoltage);
    }
}
