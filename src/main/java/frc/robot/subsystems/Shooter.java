package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


// NOT DONE, NEEDS VELOCITY REGULATION BETWEEN THE TWO MOTORS
public class Shooter extends SubsystemBase {
    private final CANSparkMax topMotor, bottomMotor;
    private final RelativeEncoder topEncoder, bottomEncoder;
    
    public static final double PEAK_RPM = 6000;

    private double targetSpeed = 0;


    public Shooter() {
        this.topMotor = new CANSparkMax(SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless);
        this.bottomMotor = new CANSparkMax(SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless);

        this.topMotor.setIdleMode(IdleMode.kCoast);
        this.bottomMotor.setIdleMode(IdleMode.kCoast);

        this.topMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT_AMPS);
        this.bottomMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT_AMPS);
    
        this.topEncoder = topMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        this.bottomEncoder = bottomMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetRPM(double targetSpeed) {
        this.targetSpeed = targetSpeed;
    }

    /**
     * @return speed of top wheel in RPM
     */
    public double getTopRPM() {
        return topEncoder.getVelocity();
    }

    /**
     * @return speed of bottom wheel in RPM
     */
    public double getBottomRPM() {
        return bottomEncoder.getVelocity();
    }
    
    @Override
    public void periodic() {
        topMotor.set(targetSpeed); // CHANGE TO FLYWHEEL STATE MODEL.
        bottomMotor.set(targetSpeed);
    }

}
