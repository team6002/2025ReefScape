package frc.robot.subsystems.CoralIntake;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CoralHolderConstants;

public class CoralHolderIOSparkMax implements CoralIHolderIO{
    private final SparkMax m_coralHolderMotor;
    private SparkBaseConfig m_coralHolderConfig;
    private final SparkClosedLoopController m_coralHolderController;

    public CoralHolderIOSparkMax(){
        //initialize motor
        m_coralHolderMotor = new SparkMax(CoralHolderConstants.kCoralHolderCanId, MotorType.kBrushless);

        //initialize PID controller
        m_coralHolderController = m_coralHolderMotor.getClosedLoopController();

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
    }
}
