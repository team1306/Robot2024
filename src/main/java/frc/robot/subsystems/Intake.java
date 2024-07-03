package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.*;

public class Intake extends NeoGroupSubsystem{

    public Intake() {
        super(
            new NeoData(MotorUtil.initSparkMax(INTAKE_FRONT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(INTAKE_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(INTAKE_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(INTAKE_CENTER_BELT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(INTAKE_CENTER_OMNI_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false)
        );
    }

    @Override
    public void periodic(){
        super.periodic();
    }
    
}
