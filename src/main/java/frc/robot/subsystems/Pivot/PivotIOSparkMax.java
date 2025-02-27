package frc.robot.subsystems.Pivot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.GlobalVariables;
import frc.robot.Configs;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.HardwareConstants;

public class PivotIOSparkMax implements PivotIO{
    private final SparkMax m_leftPivotMotor;
    private final SparkMax m_rightPivotMotor;
    private final SparkAbsoluteEncoder m_pivotEncoder;
    private final SparkClosedLoopController m_pivotController;
    private ArmFeedforward m_pivotFeedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG,
        PivotConstants.kV, PivotConstants.kA);
    private Constraints m_pivotConstraints = new Constraints(PivotConstants.kMaxVel, PivotConstants.kMaxAccel);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    double prevVelo;
    public PivotIOSparkMax(){
        m_leftPivotMotor = new SparkMax(HardwareConstants.kLeftPivotCanId, MotorType.kBrushless);
        m_rightPivotMotor = new SparkMax(HardwareConstants.kRightPivotCanId, MotorType.kBrushless);
        m_pivotEncoder = m_rightPivotMotor.getAbsoluteEncoder();
        
        m_pivotController = m_rightPivotMotor.getClosedLoopController();
        
        m_leftPivotMotor.configure(Configs.ElevatorPivotConfig.m_leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightPivotMotor.configure(Configs.ElevatorPivotConfig.m_rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_setpoint = new TrapezoidProfile.State(getPosition() - PivotConstants.kPivotOffset, 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(PivotIoInputs inputs) {
        inputs.m_pivotGoal = m_goal.position + PivotConstants.kPivotOffset;
        inputs.m_pivotPos = getPosition();
        inputs.m_pivotCurrent = getCurrent();
        inputs.m_pivotInPosition = inPosition();
        inputs.m_pivotSetpoint = m_setpoint.position+PivotConstants.kPivotOffset;
    };
    
    @Override
    public void setGoal(double p_pivotGoal){
        if(GlobalVariables.m_elevatorExtension > 5){
            m_pivotConstraints = new Constraints(PivotConstants.kMaxVelExtended, PivotConstants.kMaxAccelExtended);
        }else{
            m_pivotConstraints = new Constraints(PivotConstants.kMaxVel, PivotConstants.kMaxAccel);
        }
        m_setpoint = new TrapezoidProfile.State(getPosition() - PivotConstants.kPivotOffset, 0);
        m_goal = new TrapezoidProfile.State(p_pivotGoal - PivotConstants.kPivotOffset, 0);
    }

    @Override
    public double getGoal(){
        return m_goal.position + PivotConstants.kPivotOffset;
    }
    
    @Override
    public double getPosition(){
        return m_pivotEncoder.getPosition() + PivotConstants.kPivotOffset;
    }

    @Override
    public double getCurrent(){
        return m_rightPivotMotor.getOutputCurrent();
    }

    @Override
    public double getSetpoint(){
        return m_setpoint.position + PivotConstants.kPivotOffset;
    }

    @Override
    public boolean inPosition(){
        return Math.abs(getPosition() - getGoal()) < PivotConstants.kTolerance;
    }

    @Override
    public void PID(){
        var profile = new TrapezoidProfile(m_pivotConstraints).calculate(0.02, m_setpoint, m_goal);
        m_setpoint = profile;
        m_pivotController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_pivotFeedforward.calculate(getPosition(), m_setpoint.velocity));
    }

    @Override
    public void reset(){
        double accel = (m_setpoint.velocity - prevVelo)/.02;
        double feedforwards = m_pivotFeedforward.calculate(getPosition(), m_setpoint.velocity, accel);
        m_setpoint = new TrapezoidProfile.State(getPosition() - PivotConstants.kPivotOffset, 0);
        m_goal = m_setpoint;
        m_pivotController.setReference(m_setpoint.position, ControlType.kPosition,
            ClosedLoopSlot.kSlot0, feedforwards );
        prevVelo = m_setpoint.velocity;
    }
}
