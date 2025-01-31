package frc.robot.subsystems.ElevatorPivot;

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
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.Constants.HardwareConstants;

public class ElevatorPivotIOSparkMax implements ElevatorPivotIO{
    private final SparkMax m_leftPivotMotor;
    private final SparkMax m_rightPivotMotor;
    private final SparkAbsoluteEncoder m_pivotEncoder;
    private final SparkClosedLoopController m_pivotController;
    private ArmFeedforward m_pivotFeedforward = new ArmFeedforward(ElevatorPivotConstants.kS, ElevatorPivotConstants.kG,
        ElevatorPivotConstants.kV);
    private final Constraints m_pivotConstraints = new Constraints(ElevatorPivotConstants.kMaxVel, ElevatorPivotConstants.kMaxAccel);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    public ElevatorPivotIOSparkMax(){
        m_leftPivotMotor = new SparkMax(HardwareConstants.kLeftPivotCanId, MotorType.kBrushless);
        m_rightPivotMotor = new SparkMax(HardwareConstants.kRightPivotCanId, MotorType.kBrushless);
        m_pivotEncoder = m_rightPivotMotor.getAbsoluteEncoder();

        m_pivotController = m_rightPivotMotor.getClosedLoopController();

        
        m_leftPivotMotor.configure(Configs.ElevatorPivotConfig.m_leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightPivotMotor.configure(Configs.ElevatorPivotConfig.m_rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_setpoint = new TrapezoidProfile.State(getPosition() - ElevatorPivotConstants.kPivotOffset, 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(ElevatorPivotIoInputs inputs) {
        inputs.m_pivotGoal = m_goal.position + ElevatorPivotConstants.kPivotOffset;
        inputs.m_pivotPos = getPosition();
        inputs.m_pivotCurrent = getCurrent();
    };
    
    @Override
    public void setGoal(double p_pivotGoal){
        m_setpoint = new TrapezoidProfile.State(getPosition() - ElevatorPivotConstants.kPivotOffset, 0);
        m_goal = new TrapezoidProfile.State(p_pivotGoal - ElevatorPivotConstants.kPivotOffset, 0);
    }

    @Override
    public double getGoal(){
        return m_goal.position + ElevatorPivotConstants.kPivotOffset;
    }

    @Override
    public void setSpeed(double p_speed){
        m_rightPivotMotor.set(.1);
        m_leftPivotMotor.set(.1);
    }
    
    @Override
    public double getPosition(){
        return m_pivotEncoder.getPosition() + ElevatorPivotConstants.kPivotOffset;
    }

    @Override
    public double getCurrent(){
        return m_rightPivotMotor.getOutputCurrent();
    }

    @Override
    public void setPid(double kP, double kI, double kD, double kFF){
        Configs.ElevatorPivotConfig.m_leftPivotConfig.closedLoop.pidf(kP, kI, kD, kFF, ClosedLoopSlot.kSlot0);
        Configs.ElevatorPivotConfig.m_rightPivotConfig.closedLoop.pidf(kP, kI, kD, kFF, ClosedLoopSlot.kSlot0);

        m_leftPivotMotor.configure(Configs.ElevatorPivotConfig.m_leftPivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rightPivotMotor.configure(Configs.ElevatorPivotConfig.m_rightPivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setFeedforward(double kS, double kG, double kV){
        m_pivotFeedforward = new ArmFeedforward(kS, kG, kV);
    }

    @Override
    public void PID(){
        var profile = new TrapezoidProfile(m_pivotConstraints).calculate(0.02, m_setpoint, m_goal);
        m_setpoint = profile;
        m_pivotController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_pivotFeedforward.calculate(m_setpoint.velocity, getPosition()));
    }

    @Override
    public void reset(){
        m_setpoint = new TrapezoidProfile.State(getPosition() - ElevatorPivotConstants.kPivotOffset, 0);
        m_goal = m_setpoint;
    }
}
