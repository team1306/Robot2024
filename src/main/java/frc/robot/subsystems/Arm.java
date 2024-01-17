package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.revrobotics.SparkRelativeEncoder;

public class Arm extends PIDSubsystem {

    public static final int COUNTS_PER_REVOLUTION = 4096;

    CANSparkMax motor1 = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(1, MotorType.kBrushless);
    RelativeEncoder encoder1 = motor1.getEncoder(SparkRelativeEncoder.Type.kQuadrature, COUNTS_PER_REVOLUTION);
    RelativeEncoder encoder2 = motor2.getEncoder(SparkRelativeEncoder.Type.kQuadrature, COUNTS_PER_REVOLUTION);

    public static double kP = 1, kI = 1, kD = 1; // PLACEHOLDER
    public static double maxPower = 1;

    public static final double INITIAL_POSITION = 0;
    private Rotation2d targetAngle;

    public Arm() {
        super(new PIDController(kP, kI, kD), INITIAL_POSITION);
    }

    public Rotation2d getCurrentAngle() {
        double position = encoder1.getPosition()/COUNTS_PER_REVOLUTION*360;
        return Rotation2d.fromDegrees(position);
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        motor1.set(output * maxPower);
        motor2.follow(motor1, true);
    }

    /**
     * Gets measurement of PID system
     * 
     * @return output of current arm rotation in degrees
     * @see PIDSubsystem#getTargetAngle()
     */
    @Override
    protected double getMeasurement() {
       return getTargetAngle().getDegrees();
    }

    @Override
    public void periodic() {
        super.periodic();
        m_controller.setPID(kP, kI, kD);
    }
}