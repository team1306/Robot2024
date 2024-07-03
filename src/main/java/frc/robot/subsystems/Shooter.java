package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.MotorUtil;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends NeoGroupSubsystem{
    public Shooter(){
        super(
            new NeoData(MotorUtil.initSparkMax(SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false)
        );
    }

    @Override
    public void periodic(){
        super.periodic();
    }
}
