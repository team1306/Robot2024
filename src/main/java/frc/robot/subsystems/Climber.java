package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;

public class Climber extends SubsystemBase {
    
    private final CANSparkMax motorLeft;
    private final CANSparkMax motorRight;
    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;

    private double targetSpeed = 0;

    public Climber() {
        motorLeft = MotorUtil.initSparkMax(HANGER_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast);
        motorRight = MotorUtil.initSparkMax(HANGER_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast);
        encoderLeft = motorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        encoderLeft.setPosition(0);
        encoderLeft.setPositionConversionFactor(1D/15D);
        encoderRight = motorRight.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        motorRight.follow(motorLeft, true);
        encoderRight.setPositionConversionFactor(1D/15D);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setLeftSpeed(double targetSpeed) {
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
    }

    public void setRightSpeed(double targetSpeed) {
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
    }

    public void setTargetSpeed(double targetSpeed) {
        setLeftSpeed(targetSpeed);
        setRightSpeed(targetSpeed);
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
        SmartDashboard.putNumber("left climb", getLeftAngle().getDegrees());
        SmartDashboard.putNumber("right climb", getRightAngle().getDegrees());
        motorLeft.set(targetSpeed);
    }

    public double getLeftPosition() {
        return encoderLeft.getPosition();
    }

    public double getRightPosition() {
        return encoderRight.getPosition();
    }

    public Rotation2d getLeftAngle() {
        return Rotation2d.fromRotations(getLeftPosition());
    }

    public Rotation2d getRightAngle() {
        return Rotation2d.fromRotations(getRightPosition());
    }

    public void zeroEncoders(){
        encoderLeft.setPosition(0);
        encoderRight.setPosition(0);
    }
}
