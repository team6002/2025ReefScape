package frc.robot.subsystems.CoralIntake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class SUB_CoralIntake extends SubsystemBase{
    SparkMax m_armMotor;
    SparkMax m_firstStageMotor;
    SparkMax m_secondStageMotor;
    public SUB_CoralIntake(){
        m_armMotor = new SparkMax(CoralIntakeConstants.kArmMotorCanId, MotorType.kBrushless);
        m_firstStageMotor = new SparkMax(CoralIntakeConstants.kStageOneCanId, MotorType.kBrushless);
        m_secondStageMotor = new SparkMax(CoralIntakeConstants.kstageTwoCanId, MotorType.kBrushless);
    }
}
