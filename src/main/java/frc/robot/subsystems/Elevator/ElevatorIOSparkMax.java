package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.HardwareConstants;

public class ElevatorIOSparkMax implements ElevatorIO{
    private final SparkMax m_leftElevator;
    private final SparkMax m_rightElevator;
    private final SparkMax m_leftPivotMotor;
    private final SparkMax m_rightPivotMotor;
    private final SparkAbsoluteEncoder m_ElevatorEncoder;
    private final SparkAbsoluteEncoder m_pivotEncoder;
    private final SparkClosedLoopController m_ElevatorController;
    private final SparkClosedLoopController m_pivotController;
    private double m_elevatorGoal;
    private double m_pivotGoal;
    public ElevatorIOSparkMax(){
        //motor init
        m_leftElevator = new SparkMax(HardwareConstants.kLeftElevatorCanId, MotorType.kBrushless);
        m_rightElevator = new SparkMax(HardwareConstants.kRightElevatorCanId, MotorType.kBrushless);

        m_leftPivotMotor = new SparkMax(HardwareConstants.kLeftPivotCanId, MotorType.kBrushless);
        m_rightPivotMotor = new SparkMax(HardwareConstants.kRightPivotCanId, MotorType.kBrushless);

        //encoder init
        m_ElevatorEncoder = m_leftElevator.getAbsoluteEncoder();

        m_pivotEncoder = m_leftPivotMotor.getAbsoluteEncoder();

        //ClosedLoop controller init
        m_ElevatorController = m_leftElevator.getClosedLoopController();

        m_pivotController = m_leftPivotMotor.getClosedLoopController();

        //save config
        m_leftElevator.configure(Configs.ElevatorConfig.m_leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightElevator.configure(Configs.ElevatorConfig.m_rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_leftPivotMotor.configure(Configs.ElevatorConfig.m_leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightPivotMotor.configure(Configs.ElevatorConfig.m_leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset goal to 0 on init
        m_elevatorGoal = 0;
        m_pivotGoal = 0;
    }

    @Override
    public void updateInputs(ElevatorIoInputs inputs) {
        inputs.m_ElevatorGoal = m_elevatorGoal;
        inputs.m_ElevatorPos = getElevatorPosition();
        inputs.m_ElevatorCurrent = getElevatorCurrent();

        inputs.m_pivotGoal = m_pivotGoal;
        inputs.m_pivotPos = getPivotPosition();
        inputs.m_pivotCurrent = getPivotCurrent();
    };

    @Override
    public void setElevatorGoal(double p_ElevatorGoal){
        m_elevatorGoal = p_ElevatorGoal;
        m_ElevatorController.setReference(m_elevatorGoal, ControlType.kPosition);
    }

    public double getElevatorGoal(){
        return m_elevatorGoal;
    }

    public double getElevatorPosition(){
        return m_ElevatorEncoder.getPosition();
    }

    public double getElevatorCurrent(){
        return m_leftElevator.getOutputCurrent();
    }

    @Override
    public void setPivotGoal(double p_pivotGoal){
        m_pivotGoal = p_pivotGoal;
        m_pivotController.setReference(m_pivotGoal, ControlType.kPosition);
    }

    public double getPivotGoal(){
        return m_pivotGoal;
    }

    public double getPivotPosition(){
        return m_pivotEncoder.getPosition();
    }

    public double getPivotCurrent(){
        return m_leftPivotMotor.getOutputCurrent();
    }
}
