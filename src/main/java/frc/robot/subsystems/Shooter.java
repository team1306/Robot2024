package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;


// NOT DONE, NEEDS VELOCITY REGULATION BETWEEN THE TWO MOTORS
public class Shooter extends SubsystemBase {

    private final CANSparkMax topMotor, bottomMotor;
    private final RelativeEncoder topEncoder, bottomEncoder;
    
    public static final double PEAK_OUTPUT = 1.0;

    private double targetSpeed = 0;


    public Shooter() {
        topMotor = MotorUtil.initSparkMax(SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        bottomMotor = MotorUtil.initSparkMax(SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        
        topEncoder = topMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        bottomEncoder = bottomMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetSpeed(double targetSpeed) {
        this.targetSpeed = MathUtil.clamp(targetSpeed, 0, PEAK_OUTPUT);
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

    public Command getToggleShooterCommand(DoubleSupplier speedSupplier) {
        return new Command() {
            {
                addRequirements(Shooter.this);
            }
            @Override
            public void initialize(){
               setTargetSpeed(speedSupplier.getAsDouble()); 
            }
            @Override
            public void end(boolean interrupted){
                setTargetSpeed(0);
            }
        };
    }
}
