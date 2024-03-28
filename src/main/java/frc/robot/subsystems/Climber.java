package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.*;
@Deprecated
public class Climber extends SubsystemBase {
    
    private final CANSparkMax motorLeft;
    private final CANSparkMax motorRight;
    private final RelativeEncoder encoderLeft;
    private final RelativeEncoder encoderRight;

    private double leftTargetSpeed = 0;
    private double rightTargetSpeed = 0;

    public Climber() {
        motorLeft = MotorUtil.initSparkMax(HANGER_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast, 30);
        motorRight = MotorUtil.initSparkMax(HANGER_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast, 30);
        motorRight.setInverted(true);
        encoderLeft = motorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        
        encoderLeft.setPosition(0);
        encoderLeft.setPositionConversionFactor(1D/15D);
        encoderRight = motorRight.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        encoderRight.setPosition(0);
        encoderRight.setPositionConversionFactor(1D/15D);
    }

    public void setLeftSpeed(double targetSpeed) {
        this.leftTargetSpeed = MotorUtil.clampPercent(targetSpeed);
    }

    public void setRightSpeed(double targetSpeed) {
        this.rightTargetSpeed = MotorUtil.clampPercent(targetSpeed);
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
        motorLeft.set(leftTargetSpeed);
        motorRight.set(rightTargetSpeed);
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
