package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.MotorUtil;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends NeoGroupSubsystem{
    
    private double targetSpeed = 0;

    public Shooter(){
        super(
            new NeoData(MotorUtil.initSparkMax(SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false)
        );
        super.relativeSpeed = 0;
    }

    @Override
    public void periodic(){
        super.relativeSpeed = targetSpeed;
        super.periodic();

        SmartDashboard.putNumber("Shooter Target Speed", getTargetSpeed()); 
        //TODO GET TOP AND BOTTOM RPM
    }

    public double getTargetSpeed(){
        return targetSpeed;
    }

    public void setTargetSpeed(double targetSpeed){
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
    }
    
}
