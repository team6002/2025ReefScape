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

public class ElevatorIOSparkMax implements ElevatorIO{
    private final SparkMax m_leftLift;
    private final SparkMax m_rightLift;
    private final SparkAbsoluteEncoder m_liftEncoder;
    private final SparkClosedLoopController m_leftController;
    private SparkBaseConfig m_leftLiftConfig;
    private SparkBaseConfig m_rightLiftConfig;
    private double m_liftGoal;
    public ElevatorIOSparkMax(){
        //motor init
        m_leftLift = new SparkMax(LiftConstants.kLeftLiftCanId, MotorType.kBrushless);
        m_rightLift = new SparkMax(LiftConstants.kRightLiftCanId, MotorType.kBrushless);

        //encoder init
        m_liftEncoder = m_leftLift.getAbsoluteEncoder();

        //ClosedLoop controller init
        m_leftController = m_leftLift.getClosedLoopController();
        
        //configuration
        m_leftLiftConfig.smartCurrentLimit(40);
        m_leftLiftConfig.idleMode(IdleMode.kBrake);
        m_leftLiftConfig.voltageCompensation(12.0);
        m_leftLiftConfig.inverted(LiftConstants.kLeftInverted);
        m_leftLiftConfig.encoder.quadratureMeasurementPeriod(25);
        m_leftLiftConfig.encoder.quadratureAverageDepth(10);
        m_leftLiftConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_leftLiftConfig.closedLoop.p(LiftConstants.kP);
        m_leftLiftConfig.closedLoop.i(LiftConstants.kI);
        m_leftLiftConfig.closedLoop.d(LiftConstants.kD);
        m_leftLiftConfig.closedLoop.velocityFF(LiftConstants.kFF);
        m_leftLiftConfig.closedLoop.outputRange(LiftConstants.kMinOutput,
        LiftConstants.kMaxOutput);

        m_rightLiftConfig.follow(LiftConstants.kLeftLiftCanId);
        m_rightLiftConfig.smartCurrentLimit(40);
        m_rightLiftConfig.idleMode(IdleMode.kBrake);
        m_rightLiftConfig.voltageCompensation(12.0);
        m_rightLiftConfig.inverted(LiftConstants.kLeftInverted);
        m_rightLiftConfig.encoder.quadratureMeasurementPeriod(25);
        m_rightLiftConfig.encoder.quadratureAverageDepth(10);
        m_rightLiftConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_rightLiftConfig.closedLoop.p(LiftConstants.kP);
        m_rightLiftConfig.closedLoop.i(LiftConstants.kI);
        m_rightLiftConfig.closedLoop.d(LiftConstants.kD);
        m_rightLiftConfig.closedLoop.velocityFF(LiftConstants.kFF);
        m_rightLiftConfig.closedLoop.outputRange(LiftConstants.kMinOutput,
        LiftConstants.kMaxOutput);

        //save config
        m_leftLift.configure(m_leftLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightLift.configure(m_rightLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset goal to 0 on init
        m_liftGoal = 0;
    }

    @Override
    public void updateInputs(ElevatorIoInputs inputs) {
        inputs.m_liftGoal = m_liftGoal;
        inputs.m_liftPos = getPosition();
    };

    @Override
    public void setGoal(double p_liftGoal){
        m_liftGoal = p_liftGoal;
        m_leftController.setReference(m_liftGoal, ControlType.kPosition);
    }

    public double getGoal(){
        return m_liftGoal;
    }

    public double getPosition(){
        return m_liftEncoder.getPosition();
    }

    public double getCurrent(){
        return m_leftLift.getOutputCurrent();
    }
}
