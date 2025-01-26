package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 0;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .voltageCompensation(12)
                    .smartCurrentLimit(40);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0)// meters per second
                    .uvwMeasurementPeriod(10)
                    .uvwAverageDepth(2); 
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);
            drivingConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(false);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(30);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    .averageDepth(2);
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
            turningConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(false);
        }
    }

    public static final class ElevatorConfig{
        public static final SparkMaxConfig m_leftElevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig m_rightElevatorConfig = new SparkMaxConfig();

        static {
                m_leftElevatorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ElevatorConstants.kLeftInverted)
                        .voltageCompensation(12.0)
                        .smartCurrentLimit(40);
                m_leftElevatorConfig.closedLoop
                        .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
                m_leftElevatorConfig.encoder
                        .positionConversionFactor(ElevatorConstants.kConversionFactor);
                m_rightElevatorConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);

                m_rightElevatorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ElevatorConstants.kRightInverted)
                        .follow(HardwareConstants.kLeftElevatorCanId)
                        .voltageCompensation(12.0)
                        .smartCurrentLimit(40);
                m_rightElevatorConfig.closedLoop
                        .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
                m_rightElevatorConfig.encoder
                        .positionConversionFactor(ElevatorConstants.kConversionFactor);
                m_rightElevatorConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }

        public static final class ArmConfigs{
                public static final SparkMaxConfig m_armConfig = new SparkMaxConfig();
                static {
                        m_armConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(ArmConstants.kArmInverted)
                                .smartCurrentLimit(40)
                                .voltageCompensation(12.0);
                        m_armConfig.absoluteEncoder
                                .inverted(false)
                                .positionConversionFactor(360/(Math.PI*2))
                                .velocityConversionFactor((360/(Math.PI*2))/60)
                                .averageDepth(2);
                        m_armConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .outputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput)
                                .pidf(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kFF);
                        m_armConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }
    public static final class ElevatorPivotConfig{
        public static final SparkMaxConfig m_leftPivotConfig = new SparkMaxConfig();
        public static final SparkMaxConfig m_rightPivotConfig = new SparkMaxConfig();

        static{
                m_leftPivotConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ElevatorPivotConstants.kLeftInverted)
                        .voltageCompensation(12.0)
                        .follow(HardwareConstants.kRightPivotCanId,true)
                        .smartCurrentLimit(40);
                m_leftPivotConfig.closedLoop
                        .pid(ElevatorPivotConstants.kP, ElevatorPivotConstants.kI, ElevatorPivotConstants.kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ElevatorPivotConstants.kMinOutput, ElevatorPivotConstants.kMaxOutput);
                m_leftPivotConfig.encoder
                        .positionConversionFactor(ElevatorPivotConstants.kConversionFactor)
                        .uvwMeasurementPeriod(10)
                        .uvwAverageDepth(2); 
                
                m_rightPivotConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);

                m_rightPivotConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ElevatorPivotConstants.kRightInverted)
                        .voltageCompensation(12.0)
                        .disableFollowerMode()
                        .smartCurrentLimit(40);
                m_rightPivotConfig.closedLoop
                        .pid(ElevatorPivotConstants.kP, ElevatorPivotConstants.kI, ElevatorPivotConstants.kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ElevatorPivotConstants.kMinOutput, ElevatorPivotConstants.kMaxOutput);
                m_rightPivotConfig.encoder
                        .positionConversionFactor(ElevatorPivotConstants.kConversionFactor)
                        .uvwMeasurementPeriod(10)
                        .uvwAverageDepth(2); 
                
                m_rightPivotConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }    
}
