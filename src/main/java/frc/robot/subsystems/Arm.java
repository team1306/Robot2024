package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.util.MotorUtil;

import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.*;

public class Arm extends PIDSubsystem {
    public enum ControlMode {
        MANUAL,
        AUTOMATIC
    }

    private final CANSparkMax motor1;
    private final CANSparkMax motor2;
    private final DutyCycleEncoder throughBoreEncoder;
    
    private final RelativeEncoder neoEncoder;

    private final ArmFeedforward feedforward;

    public static double kP = 1, kI = 0, kD = 0; // Do we want PID Controller? Or do we want to do state space model?
                                                 // need to read https://file.tavsys.net/control/controls-engineering-in-frc.pdf more so I know what I am doing
    public static double kS = 1, kG = 1, kA = 1, kV = 1; // PLACEHOLDER

    public static final double INITIAL_POSITION = 0;

    public static double holdPower = 0.1;

    private Rotation2d targetAngle;
    private double power;

    private long velocityIndex = 0;
    private double[] velocities = new double[2];

    private ControlMode controlMode, lastControlMode;

    public Arm(ControlMode controlMode) {
        super(new PIDController(kP, kI, kD), INITIAL_POSITION);
        
        motor1 = MotorUtil.initSparkMax(ARM_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        motor2 = MotorUtil.initSparkMax(ARM_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        motor1.setInverted(false);
        motor2.setInverted(true);
        
        throughBoreEncoder = new DutyCycleEncoder(0);
        throughBoreEncoder.setDistancePerRotation(1/100); // ARM GEAR RATIO

        neoEncoder = motor2.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        neoEncoder.setPosition(0);
        neoEncoder.setPositionConversionFactor(Units.rotationsToRadians(1)); // REVOLUTIONS -> RAD
        neoEncoder.setVelocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1/100)); // RPM -> RAD/SEC
    

        feedforward = new ArmFeedforward(kS, kG, kV, kA);

        this.controlMode = controlMode;

        SmartDashboard.putNumber("Hold Power", 0.1);
    }
    
    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    public Arm() {
        this(ControlMode.MANUAL);
    }
        
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(throughBoreEncoder.getDistance());
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }

    private int calcVelocityIndex(long index) {
        return (int) (index % velocities.length);
    }

    private double calculateAcceleration() {
        return (velocities[calcVelocityIndex(velocityIndex)] - velocities[calcVelocityIndex(velocityIndex + 1)]) / LOOP_TIME_SECONDS; 
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        output += feedforward.calculate(getCurrentAngle().getRadians(), neoEncoder.getVelocity(), calculateAcceleration());
        motor1.setVoltage(MathUtil.clamp(output, -NEO_MAX_VOLTAGE, NEO_MAX_VOLTAGE));
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

    public void setManualPower(double power) {
        this.power = Math.max(holdPower, power);
    }

    @Override
    public void periodic() {
        holdPower = SmartDashboard.getNumber("Hold Power", 0.1);
        if (lastControlMode != controlMode && controlMode == ControlMode.MANUAL) {
            setManualPower(0);
        }
        m_controller.setPID(kP, kI, kD);
        velocities[calcVelocityIndex(velocityIndex)] = neoEncoder.getVelocity();
        switch (controlMode){
            case AUTOMATIC:
                super.periodic();
            case MANUAL:
                motor1.set(power);
                motor2.set(power);
                SmartDashboard.putNumber("right arm power", motor2.get());
                SmartDashboard.putNumber("left arm power", motor1.get());
        }
        ++velocityIndex;
        lastControlMode = controlMode;
    }
}