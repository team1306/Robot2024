package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends NeoGroupSubsystem{

    private double targetSpeed = 0;

    public Indexer(){
        super(new NeoData(MotorUtil.initSparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false));
        super.relativeSpeed = 0;
    }

    @Override
    public void periodic(){
        super.relativeSpeed = targetSpeed;
        super.periodic();

        SmartDashboard.putNumber("Indexer Target Speed", getTargetSpeed());
    }

    public double getTargetSpeed(){
        return targetSpeed;
    }

    public void setTargetSpeed(double targetSpeed){
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
    }
}
