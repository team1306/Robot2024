package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;

public class Intake extends SubsystemBase {
    
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput sensor;

    private double targetSpeed = 0;
    private boolean sensorReading = false;

    public Intake() {
        motor = MotorUtil.initSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        sensor = new DigitalInput(1);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetSpeed(double targetSpeed) {
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
        System.out.println("Setting intake target speed to " + targetSpeed);
    }

    /**
     * @return speed of the wheel in RPM
     */
    public double getRPM() {
        return encoder.getVelocity();
    }
    
    @Override
    public void periodic() {
        motor.set(targetSpeed);
        sensorReading = sensor.get();
        SmartDashboard.putBoolean("Current Intake Sensor Reading", sensorReading);
    }

    public boolean notePresent() {
        return sensorReading;
    }
}
