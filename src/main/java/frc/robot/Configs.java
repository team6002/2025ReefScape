package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.*;

public final class Configs {
        public static final class MAXSwerveModule {
                public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                double turningFactor = 2 * Math.PI;
                double drivingVelocityFeedForward = 0;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .voltageCompensation(12)
                    .smartCurrentLimit(130)
                    .inverted(false);
            drivingConfig.encoder
                    .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor) // meters
                    .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)// meters per second
                    .uvwMeasurementPeriod(10)
                    .uvwAverageDepth(2); 
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD,ClosedLoopSlot.kSlot0)
                    .pid(ModuleConstants.kAutoP, ModuleConstants.kAutoI, ModuleConstants.kAutoD,ClosedLoopSlot.kSlot1)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);
            drivingConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(false);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
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
                                .follow(HardwareConstants.kRightElevatorCanId, true)
                                .voltageCompensation(12.0)
                                .smartCurrentLimit(80);
                        m_leftElevatorConfig.closedLoop
                                .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
                        m_leftElevatorConfig.encoder
                                .uvwAverageDepth(2)
                                .uvwMeasurementPeriod(10)
                                .positionConversionFactor(ElevatorConstants.kConversionFactor);
                        m_leftElevatorConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);

                        m_rightElevatorConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(ElevatorConstants.kRightInverted)
                                .disableFollowerMode()
                                .voltageCompensation(12.0)
                                .smartCurrentLimit(80);
                        m_rightElevatorConfig.closedLoop
                                .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
                        m_rightElevatorConfig.encoder
                                .uvwAverageDepth(2)
                                .uvwMeasurementPeriod(10)
                                .positionConversionFactor(ElevatorConstants.kConversionFactor);
                        m_rightElevatorConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }

        public static final class FlippyWristConfigs{
                public static final SparkMaxConfig m_flippyWristConfig = new SparkMaxConfig();
                static {
                        m_flippyWristConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(FlippyWristConstants.kWristInverted)
                                .smartCurrentLimit(40)
                                .disableFollowerMode()
                                .voltageCompensation(12.0);
                        m_flippyWristConfig.absoluteEncoder
                                .inverted(true)
                                .positionConversionFactor(FlippyWristConstants.kConversionFactor)
                                .velocityConversionFactor(FlippyWristConstants.kConversionFactor/60)
                                .averageDepth(2);
                        m_flippyWristConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .outputRange(FlippyWristConstants.kMinOutput, FlippyWristConstants.kMaxOutput)
                                .pidf(FlippyWristConstants.kP, FlippyWristConstants.kI, FlippyWristConstants.kD, FlippyWristConstants.kFF)
                                .positionWrappingInputRange(Math.toRadians(0), FlippyWristConstants.kConversionFactor)
                                .positionWrappingEnabled(true);
                        m_flippyWristConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }

        public static final class SpinnyWristConfigs{
                public static final SparkMaxConfig m_spinnyWristConfig = new SparkMaxConfig();
                static {
                        m_spinnyWristConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(FlippyWristConstants.kWristInverted)
                                .smartCurrentLimit(40)
                                .disableFollowerMode()
                                .voltageCompensation(12.0);
                        m_spinnyWristConfig.absoluteEncoder
                                .inverted(true)
                                .positionConversionFactor(FlippyWristConstants.kConversionFactor)
                                .velocityConversionFactor(FlippyWristConstants.kConversionFactor/60)
                                .averageDepth(2);
                        m_spinnyWristConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .outputRange(FlippyWristConstants.kMinOutput, FlippyWristConstants.kMaxOutput)
                                .pidf(FlippyWristConstants.kP, FlippyWristConstants.kI, FlippyWristConstants.kD, FlippyWristConstants.kFF)
                                .positionWrappingInputRange(Math.toRadians(0), FlippyWristConstants.kConversionFactor)
                                .positionWrappingEnabled(true);
                        m_spinnyWristConfig.limitSwitch
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
                                .inverted(PivotConstants.kLeftInverted)
                                .voltageCompensation(12.0)
                                .follow(HardwareConstants.kRightPivotCanId,true)
                                .smartCurrentLimit(80);
                        m_leftPivotConfig.closedLoop
                                .pid(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .outputRange(PivotConstants.kMinOutput, PivotConstants.kMaxOutput);
                        m_leftPivotConfig.encoder
                                .positionConversionFactor(PivotConstants.kConversionFactor)
                                .uvwMeasurementPeriod(10)
                                .uvwAverageDepth(2); 
                        m_leftPivotConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);

                        m_rightPivotConfig
                                .idleMode(IdleMode.kBrake)//Brake
                                .inverted(PivotConstants.kRightInverted)
                                .voltageCompensation(12.0)
                                .disableFollowerMode()
                                .smartCurrentLimit(80);
                        m_rightPivotConfig.closedLoop
                                .pid(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD)
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .outputRange(PivotConstants.kMinOutput, PivotConstants.kMaxOutput);
                        m_rightPivotConfig.absoluteEncoder
                                .averageDepth(2)
                                .positionConversionFactor(PivotConstants.kConversionFactor);
                        m_rightPivotConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }   
    
        public static final class IntakeConfig{
                public static final SparkMaxConfig m_intakeConfig = new SparkMaxConfig();
                static{
                        m_intakeConfig
                                .disableFollowerMode()
                                .idleMode(IdleMode.kBrake)
                                .inverted(IntakeConstants.kInverted)
                                .smartCurrentLimit(40)
                                .voltageCompensation(12.0);
                        m_intakeConfig.encoder
                                .quadratureAverageDepth(2)
                                .quadratureMeasurementPeriod(10);
                        m_intakeConfig.closedLoop
                                .pidf(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kFF)
                                .outputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                        m_intakeConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }

        public static final class WinchConfig{
                public static final SparkMaxConfig m_WinchConfig = new SparkMaxConfig();

                static{
                        m_WinchConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .disableFollowerMode()
                                .voltageCompensation(12.0)
                                .smartCurrentLimit(40);
                        m_WinchConfig.closedLoop
                                .pidf(WinchConstants.kP, WinchConstants.kI, WinchConstants.kD, WinchConstants.kFF)
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .outputRange(AlgaeConstants.kMinOutput, AlgaeConstants.kMaxOutput);
                        m_WinchConfig.absoluteEncoder
                                .averageDepth(2)
                                .inverted(false)
                                .positionConversionFactor(360);
                        m_WinchConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }

        public static final class AlgaeConfig{
                public static final SparkMaxConfig m_AlgaeConfig = new SparkMaxConfig();

                static{
                        m_AlgaeConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .disableFollowerMode()
                                .voltageCompensation(12.0)
                                .smartCurrentLimit(40);
                        m_AlgaeConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .outputRange(WinchConstants.kMinOutput, WinchConstants.kMaxOutput);
                        m_AlgaeConfig.encoder
                                .uvwAverageDepth(2)
                                .uvwMeasurementPeriod(10)
                                .positionConversionFactor(360)
                                .velocityConversionFactor(1);
                        m_AlgaeConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }

        public static final class GroundPivotConfig{
                public static final SparkMaxConfig m_groundPivotConfig = new SparkMaxConfig();
                static {
                        m_groundPivotConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(GroundPivotConstants.kInverted)
                                .smartCurrentLimit(40)
                                .disableFollowerMode()
                                .voltageCompensation(12.0);
                        m_groundPivotConfig.absoluteEncoder
                                .inverted(true)
                                .positionConversionFactor(GroundPivotConstants.kConversionFactor)
                                .velocityConversionFactor(GroundPivotConstants.kConversionFactor/60)
                                .averageDepth(2);
                        m_groundPivotConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .outputRange(GroundPivotConstants.kMinOutput, GroundPivotConstants.kMaxOutput)
                                .pidf(GroundPivotConstants.kP, GroundPivotConstants.kI, GroundPivotConstants.kD, GroundPivotConstants.kFF);
                        m_groundPivotConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }

        public static final class GroundIntake{
                public static final SparkMaxConfig m_groundIntakeConfig = new SparkMaxConfig();

                static{
                        m_groundIntakeConfig
                                .disableFollowerMode()
                                .idleMode(IdleMode.kBrake)
                                .inverted(GroundIntakeConstants.kInverted)
                                .smartCurrentLimit(40)
                                .voltageCompensation(12.0);
                        m_groundIntakeConfig.encoder
                                .quadratureAverageDepth(2)
                                .quadratureMeasurementPeriod(10);
                        m_groundIntakeConfig.closedLoop
                                .pidf(GroundIntakeConstants.kP, GroundIntakeConstants.kI, GroundIntakeConstants.kD, GroundIntakeConstants.kFF)
                                .outputRange(GroundIntakeConstants.kMinOutput, GroundIntakeConstants.kMaxOutput)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                        m_groundIntakeConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }
}
