package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;

public class Intake extends SubsystemBase {
    
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput sensor;
    public static final double MAX_RPM = 6000;

    private double targetSpeed = 0;
    private boolean sensorReading = false;

    public Intake() {
        motor = MotorUtil.initSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast);
        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        sensor = new DigitalInput(1);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetRPM(double targetSpeed) {
        this.targetSpeed = MathUtil.clamp(targetSpeed, 0, MAX_RPM);
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
        SmartDashboard.putBoolean("Current Sensor Reading", sensorReading);
    }

    public boolean notePresent() {
        return sensorReading;
    }
}
