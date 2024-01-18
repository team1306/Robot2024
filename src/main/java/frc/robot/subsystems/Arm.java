package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.*;

public class Arm extends PIDSubsystem {
    private final CANSparkMax motor1;
    private final CANSparkMax motor2;
    private final RelativeEncoder encoder;

    private final ArmFeedforward feedforward;

    public static double kP = 1, kI = 0, kD = 0; // Do we want PID Controller? Or do we want to do state space model?
                                                 // need to read https://file.tavsys.net/control/controls-engineering-in-frc.pdf more so I know what I am doing
    public static double kS = 1, kG = 1, kA = 1, kV = 1; // PLACEHOLDER

    public static final double INITIAL_POSITION = 0;
    private Rotation2d targetAngle;

    private long velocityIndex = 0;
    private double[] velocities = new double[2];


    public Arm() {
        super(new PIDController(kP, kI, kD), INITIAL_POSITION);
        motor1 = new CANSparkMax(ARM_LEFT_MOTOR_ID, MotorType.kBrushless);
        motor2 = new CANSparkMax(ARM_RIGHT_MOTOR_ID, MotorType.kBrushless);

        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();

        motor1.setIdleMode(IdleMode.kBrake);
        motor2.setIdleMode(IdleMode.kBrake);
        motor1.setSmartCurrentLimit(NEO_CURRENT_LIMIT_AMPS);
        motor2.setSmartCurrentLimit(NEO_CURRENT_LIMIT_AMPS);

        motor1.follow(motor2, true);
        
        encoder = motor2.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        encoder.setPosition(0);
        encoder.setPositionConversionFactor(Units.rotationsToRadians(1)); // REVOLUTIONS -> RAD
        encoder.setVelocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1)); // RPM -> RAD/SEC

        feedforward = new ArmFeedforward(kS, kG, kV, kA);
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(encoder.getPosition());
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }

    private int calcVelocityIndex(long index) {
        return (int) (index % velocities.length);
    }

    private double calculateAcceleration() {
        return (velocities[calcVelocityIndex(velocityIndex)] - velocities[calcVelocityIndex(velocityIndex + 1)]) / LOOP_SECONDS; 
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        output += feedforward.calculate(getCurrentAngle().getRadians(), encoder.getVelocity(), calculateAcceleration());
        motor2.setVoltage(MathUtil.clamp(output, -NEO_MAX_VOLTAGE, NEO_MAX_VOLTAGE)); 
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
        m_controller.setPID(kP, kI, kD);
        // LIVE UPDATE ARM FEEDFORWARD FOR DEBUG SOMEHOW HERE

        velocities[calcVelocityIndex(velocityIndex)] = encoder.getVelocity();
        super.periodic();

        ++velocityIndex;
    }
}