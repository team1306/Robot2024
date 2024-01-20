package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;


// NOT DONE, NEEDS VELOCITY REGULATION BETWEEN THE TWO MOTORS
public class Shooter extends SubsystemBase {
    private final CANSparkMax topMotor, bottomMotor;
    private final RelativeEncoder topEncoder, bottomEncoder;
    
    public static final double PEAK_RPM = 6000;

    private double targetSpeed = 0;


    public Shooter() {
        topMotor = MotorUtil.initSparkMax(SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast);
        bottomMotor = MotorUtil.initSparkMax(SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast);
        
        topEncoder = topMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        bottomEncoder = bottomMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
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
