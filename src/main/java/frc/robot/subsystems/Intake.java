package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.*;

public class Intake extends NeoGroupSubsystem{

    private double targetSpeed = 0;

    public Intake() {
        super(
            new NeoData(MotorUtil.initSparkMax(INTAKE_FRONT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(INTAKE_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(INTAKE_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(INTAKE_CENTER_BELT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false),
            new NeoData(MotorUtil.initSparkMax(INTAKE_CENTER_OMNI_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false)
        );
        super.relativeSpeed = 0;

        //TODO INCLUDE NOTE SENSOR
    }

    @Override
    public void periodic(){
        super.relativeSpeed = targetSpeed;
        super.periodic();

        SmartDashboard.putNumber("Intake Target Speed", getTargetSpeed());
    }

    public double getTargetSpeed(){
        return targetSpeed;
    }

    public void setTargetSpeed(double targetSpeed){
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
    }
}
