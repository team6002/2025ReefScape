package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HardwareConstants;

public class ArmIOSparkMax implements ArmIO{
    private final SparkMax m_armMotor;
    private final AbsoluteEncoder m_armEncoder;
    private final SparkClosedLoopController m_armController;
    private final SimpleMotorFeedforward m_armFeedforward = new SimpleMotorFeedforward(ArmConstants.kS, ArmConstants.kV);
    private final Constraints m_armConstraints = new Constraints(ArmConstants.kMaxVel, ArmConstants.kMaxAccel);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    public ArmIOSparkMax(){
        //initialize motor
        m_armMotor = new SparkMax(HardwareConstants.kCoralHolderCanId, MotorType.kBrushless);

        //initialize PID controller
        m_armController = m_armMotor.getClosedLoopController();

        //initalize encoder
        m_armEncoder = m_armMotor.getAbsoluteEncoder();

        //apply config
        m_armMotor.configure(Configs.ArmConfigs.m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset reference in init
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        inputs.m_armReference = getReference();
        inputs.m_armCurrent = getCurrent();
        inputs.m_armPosition = getReference();
    }

    @Override
    public void setReference(double p_reference){
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = new TrapezoidProfile.State(p_reference, 0);
    }

    public double getPosition(){
        return m_armEncoder.getPosition();
    }

    public double getCurrent(){
        return m_armMotor.getOutputCurrent();
    }

    public double getReference(){
        return m_goal.position;
    }

    @Override
    public void PID(){
        var profile = new TrapezoidProfile(m_armConstraints).calculate(0.02, m_setpoint, m_goal);
        m_armController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_armFeedforward.calculate(m_setpoint.velocity));
    }
}
