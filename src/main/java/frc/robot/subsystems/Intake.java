package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;

public class Intake extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private double targetSpeed = 0;

    public Intake() {
        motor = MotorUtil.initSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast);
        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetRPM(double targetSpeed) {
        this.targetSpeed = targetSpeed;
    }

    /**
     * @return speed of the wheel in RPM
     */
    public double getRPM() {
        return encoder.getVelocity();
    }
    
    @Override
    public void periodic() {
        motor.set(targetSpeed); // CHANGE TO FLYWHEEL STATE MODEL.
    }

}