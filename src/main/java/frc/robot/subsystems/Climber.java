package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;

public class Climber extends SubsystemBase {
    
    private final CANSparkMax motorLeft;
    private final CANSparkMax motorRight;
    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;
    public static final double MAX_RPM = 500;

    private double targetSpeed = 0;

    public Climber() {
        motorLeft = MotorUtil.initSparkMax(HANGER_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        motorRight = MotorUtil.initSparkMax(HANGER_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        encoderLeft = motorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        encoderRight = motorRight.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        motorRight.follow(motorLeft, true);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetRPM(double targetSpeed) {
        this.targetSpeed = MathUtil.clamp(targetSpeed / MAX_RPM, -1, 1);
    }

    /**
     * @return speed of the left motor in RPM
     */
    public double getLeftRPM() {
        return encoderLeft.getVelocity();
    }

    /**
     * @return speed of the left motor in RPM
     */
    public double getRightRPM() {
        return encoderRight.getVelocity();
    }
    
    @Override
    public void periodic() {
        motorLeft.set(targetSpeed);
    }
}
