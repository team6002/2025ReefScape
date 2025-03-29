package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.HardwareConstants;

public class IntakeIOSparkMax implements IntakeIO{
    private final SparkMax m_intakeMotor;
    private final RelativeEncoder m_intakeEncoder;
    private final SparkClosedLoopController m_intakeController;
    private final SparkMax m_conveyorMotor;
    private final SparkClosedLoopController m_conveyorController;

    private double m_intakeReference;

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
        m_conveyorMotor.configure(Configs.IntakeConfig.m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset target speed in init
        m_intakeReference = 0;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.m_intakeReference = getReference();
        inputs.m_intakeCurrent = getCurrent();
        inputs.m_intakeVelocity = getVelocity();
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
    public void setVoltage(double p_voltage){
        m_intakeController.setReference(p_voltage, ControlType.kVoltage);
    }

    @Override
    public void setConveyorVoltage(double p_voltage){
        m_conveyorController.setReference(p_voltage, ControlType.kVoltage);
    }
}
