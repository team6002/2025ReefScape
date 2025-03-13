package frc.robot.subsystems.GroundPivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Configs;
import frc.robot.Constants.GroundPivotConstants;
import frc.robot.Constants.HardwareConstants;

public class GroundPivotIOSparkMax implements GroundPivotIO{
    private final SparkMax m_groundPivotMotor;
    private final AbsoluteEncoder m_groundPivotEncoder;
    private final SparkClosedLoopController m_groundPivotController;
    private  ArmFeedforward m_groundPivotFeedforward = new ArmFeedforward(GroundPivotConstants.kS, GroundPivotConstants.kG, GroundPivotConstants.kV);
    private Constraints m_groundPivotFeedConstraints = new Constraints(GroundPivotConstants.kMaxVel, GroundPivotConstants.kMaxAccel);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    public GroundPivotIOSparkMax(){
        //initialize motor
        m_groundPivotMotor = new SparkMax(HardwareConstants.kGroundPivotCanId, MotorType.kBrushless);

        //initialize PID controller
        m_groundPivotController = m_groundPivotMotor.getClosedLoopController();

        //initalize encoder
        m_groundPivotEncoder = m_groundPivotMotor.getAbsoluteEncoder();

        //apply config
        m_groundPivotMotor.configure(Configs.GroundPivotConfig.m_groundPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset reference in init
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(GroundPivotIOInputs inputs){
        inputs.m_groundPivotGoal = Math.toDegrees(getGoal());
        inputs.m_groundPivotCurrent = getCurrent();
        inputs.m_groundPivotPosition = Math.toDegrees(getPosition());
        inputs.m_groundPivotInPosition = inPosition();
        inputs.m_groundPivotSetpoint = Math.toDegrees(getSetpoint());
    }

    @Override
    public void setGoal(double p_Goal){
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = new TrapezoidProfile.State(p_Goal, 0);
    }

    @Override
    public double getPosition(){
        return m_groundPivotEncoder.getPosition();
    }

    @Override
    public double getCurrent(){
        return m_groundPivotMotor.getOutputCurrent();
    }

    @Override
    public double getGoal(){
        return m_goal.position;
    }

    @Override 
    public double getSetpoint(){
        return m_setpoint.position;
    }

    @Override
    public boolean inPosition(){
        return Math.abs(getPosition() - getGoal()) < GroundPivotConstants.kTolerance;
    }

    @Override
    public void PID(){
        var profile = new TrapezoidProfile(m_groundPivotFeedConstraints).calculate(0.02, m_setpoint, m_goal);
        m_setpoint = profile;
        m_groundPivotController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_groundPivotFeedforward.calculate(getPosition(), m_setpoint.velocity));
    }

    @Override
    public void setConstraints(double velocity, double acceleration){
        m_groundPivotFeedConstraints = new Constraints(velocity, acceleration);

    }
    @Override
    public void reset(){
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = m_setpoint;
        m_groundPivotController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_groundPivotFeedforward.calculate(getPosition(), m_setpoint.velocity));
    }
}
