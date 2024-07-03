package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Turret extends NeoGroupSubsystem{
    public Turret() {
        super(new NeoData(MotorUtil.initSparkMax(TURRET_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake), INCLUDE_AUTO));
    }
}
