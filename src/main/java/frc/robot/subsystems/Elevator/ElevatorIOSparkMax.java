package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.HardwareConstants;

public class ElevatorIOSparkMax implements ElevatorIO{
    private final SparkMax m_leftLift;
    private final SparkMax m_rightLift;
    private final SparkMax m_leftPivotMotor;
    private final SparkMax m_rightPivotMotor;
    private final SparkAbsoluteEncoder m_liftEncoder;
    private final SparkAbsoluteEncoder m_pivotEncoder;
    private final SparkClosedLoopController m_liftController;
    private final SparkClosedLoopController m_pivotController;
    private SparkBaseConfig m_leftLiftConfig;
    private SparkBaseConfig m_rightLiftConfig;
    private SparkBaseConfig m_leftPivotConfig;
    private SparkBaseConfig m_rightPivotConfig;
    private double m_liftGoal;
    private double m_pivotGoal;
    public ElevatorIOSparkMax(){
        //motor init
        m_leftLift = new SparkMax(HardwareConstants.kLeftLiftCanId, MotorType.kBrushless);
        m_rightLift = new SparkMax(HardwareConstants.kRightLiftCanId, MotorType.kBrushless);

        m_leftPivotMotor = new SparkMax(HardwareConstants.kLeftPivotCanId, MotorType.kBrushless);
        m_rightPivotMotor = new SparkMax(HardwareConstants.kRightPivotCanId, MotorType.kBrushless);

        //encoder init
        m_liftEncoder = m_leftLift.getAbsoluteEncoder();

        m_pivotEncoder = m_leftPivotMotor.getAbsoluteEncoder();

        //ClosedLoop controller init
        m_liftController = m_leftLift.getClosedLoopController();

        m_pivotController = m_leftPivotMotor.getClosedLoopController();
        
        //configuration
        m_leftLiftConfig.smartCurrentLimit(40);
        m_leftLiftConfig.idleMode(IdleMode.kBrake);
        m_leftLiftConfig.voltageCompensation(12.0);
        m_leftLiftConfig.inverted(LiftConstants.kLeftInverted);
        m_leftLiftConfig.encoder.quadratureMeasurementPeriod(25);
        m_leftLiftConfig.encoder.quadratureAverageDepth(10);
        m_leftLiftConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_leftLiftConfig.closedLoop.p(LiftConstants.kLiftP);
        m_leftLiftConfig.closedLoop.i(LiftConstants.kLiftI);
        m_leftLiftConfig.closedLoop.d(LiftConstants.kLiftD);
        m_leftLiftConfig.closedLoop.velocityFF(LiftConstants.kLiftFF);
        m_leftLiftConfig.closedLoop.outputRange(LiftConstants.kMinOutput,
        LiftConstants.kMaxOutput);

        m_rightLiftConfig.follow(HardwareConstants.kLeftLiftCanId);
        m_rightLiftConfig.smartCurrentLimit(40);
        m_rightLiftConfig.idleMode(IdleMode.kBrake);
        m_rightLiftConfig.voltageCompensation(12.0);
        m_rightLiftConfig.inverted(LiftConstants.kRightInverted);
        m_rightLiftConfig.encoder.quadratureMeasurementPeriod(25);
        m_rightLiftConfig.encoder.quadratureAverageDepth(10);
        m_rightLiftConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_rightLiftConfig.closedLoop.p(LiftConstants.kLiftP);
        m_rightLiftConfig.closedLoop.i(LiftConstants.kLiftI);
        m_rightLiftConfig.closedLoop.d(LiftConstants.kLiftD);
        m_rightLiftConfig.closedLoop.velocityFF(LiftConstants.kLiftFF);
        m_rightLiftConfig.closedLoop.outputRange(LiftConstants.kMinOutput,
        LiftConstants.kMaxOutput);

        m_leftPivotConfig.smartCurrentLimit(40);
        m_leftPivotConfig.idleMode(IdleMode.kBrake);
        m_leftPivotConfig.voltageCompensation(12.0);
        m_leftPivotConfig.inverted(LiftConstants.kLeftPivotInverted);
        m_leftPivotConfig.encoder.quadratureMeasurementPeriod(25);
        m_leftPivotConfig.encoder.quadratureAverageDepth(10);
        m_leftPivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_leftPivotConfig.closedLoop.p(LiftConstants.kPivotP);
        m_leftPivotConfig.closedLoop.i(LiftConstants.kPivotI);
        m_leftPivotConfig.closedLoop.d(LiftConstants.kPivotD);
        m_leftPivotConfig.closedLoop.velocityFF(LiftConstants.kPivotFF);
        m_leftPivotConfig.closedLoop.outputRange(LiftConstants.kMinOutput,
        LiftConstants.kMaxOutput);

        m_rightPivotConfig.smartCurrentLimit(40);
        m_rightPivotConfig.idleMode(IdleMode.kBrake);
        m_rightPivotConfig.voltageCompensation(12.0);
        m_rightPivotConfig.inverted(LiftConstants.kRightPivotInverted);
        m_rightPivotConfig.encoder.quadratureMeasurementPeriod(25);
        m_rightPivotConfig.encoder.quadratureAverageDepth(10);
        m_rightPivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_rightPivotConfig.closedLoop.p(LiftConstants.kPivotP);
        m_rightPivotConfig.closedLoop.i(LiftConstants.kPivotI);
        m_rightPivotConfig.closedLoop.d(LiftConstants.kPivotD);
        m_rightPivotConfig.closedLoop.velocityFF(LiftConstants.kPivotFF);
        m_rightPivotConfig.closedLoop.outputRange(LiftConstants.kMinOutput,
        LiftConstants.kMaxOutput);

        //save config
        m_leftLift.configure(m_leftLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightLift.configure(m_rightLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_leftPivotMotor.configure(m_leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightPivotMotor.configure(m_rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset goal to 0 on init
        m_liftGoal = 0;
        m_pivotGoal = 0;
    }

    @Override
    public void updateInputs(ElevatorIoInputs inputs) {
        inputs.m_liftGoal = m_liftGoal;
        inputs.m_liftPos = getLiftPosition();
        inputs.m_liftCurrent = getLiftCurrent();

        inputs.m_pivotGoal = m_pivotGoal;
        inputs.m_pivotPos = getPivotPosition();
        inputs.m_pivotCurrent = getPivotCurrent();
    };

    @Override
    public void setLiftGoal(double p_liftGoal){
        m_liftGoal = p_liftGoal;
        m_liftController.setReference(m_liftGoal, ControlType.kPosition);
    }

    public double getLiftGoal(){
        return m_liftGoal;
    }

    public double getLiftPosition(){
        return m_liftEncoder.getPosition();
    }

    public double getLiftCurrent(){
        return m_leftLift.getOutputCurrent();
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
