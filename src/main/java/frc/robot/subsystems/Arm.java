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

    private final CANSparkMax leftArmMotor;
    private final CANSparkMax rightArmMotor;
    private final DutyCycleEncoder throughBoreEncoder;
    
    private final RelativeEncoder neoEncoder;

    private ArmFeedforward feedforward;

    public static double kP = 1, kI = 0, kD = 0; // Do we want PID Controller? Or do we want to do state space model?
                                                 // need to read https://file.tavsys.net/control/controls-engineering-in-frc.pdf more so I know what I am doing
    public static double kS = 1, kG = 1, kA = 1, kV = 1; // PLACEHOLDER

    public static final double INITIAL_POSITION = 0;

    private Rotation2d targetAngle;
    private double power;

    private long velocityIndex = 0;
    private double[] velocities = new double[2];

    private ControlMode controlMode, lastControlMode;

    public Arm(ControlMode controlMode) {
        super(new PIDController(kP, kI, kD), INITIAL_POSITION);
        
        leftArmMotor = MotorUtil.initSparkMax(ARM_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        rightArmMotor = MotorUtil.initSparkMax(ARM_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        leftArmMotor.setInverted(false);
        rightArmMotor.setInverted(true);
        
        throughBoreEncoder = new DutyCycleEncoder(0);
        throughBoreEncoder.setDistancePerRotation(1/100); // ARM GEAR RATIO

        neoEncoder = rightArmMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        neoEncoder.setPosition(0);
        neoEncoder.setPositionConversionFactor(Units.rotationsToRadians(1)); // REVOLUTIONS -> RAD
        neoEncoder.setVelocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1/100)); // RPM -> RAD/SEC
    

        feedforward = new ArmFeedforward(kS, kG, kV, kA);
    
        this.controlMode = controlMode;
        this.lastControlMode = controlMode;

        SmartDashboard.putNumber("Arm kP", 1);
        SmartDashboard.putNumber("Arm kI", 0);
        SmartDashboard.putNumber("Arm kD", 0);

        SmartDashboard.putNumber("Arm kS", 1);
        SmartDashboard.putNumber("Arm kG", 1);
        SmartDashboard.putNumber("Arm kV", 1);
        SmartDashboard.putNumber("Arm kA", 1);

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
        leftArmMotor.setVoltage(MathUtil.clamp(output, -NEO_MAX_VOLTAGE, NEO_MAX_VOLTAGE));
        rightArmMotor.setVoltage(MathUtil.clamp(output, -NEO_MAX_VOLTAGE, NEO_MAX_VOLTAGE)); 
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
        this.power = power;
    }

    @Override
    public void periodic() {
        kP = SmartDashboard.getNumber("Arm kP", 1);
        kI = SmartDashboard.getNumber("Arm kI", 0);
        kD = SmartDashboard.getNumber("Arm kD",0);

        kS = SmartDashboard.getNumber("Arm kS", 1);
        kG = SmartDashboard.getNumber("Arm kG", 1);
        kV = SmartDashboard.getNumber("Arm kV", 1);
        kA = SmartDashboard.getNumber("Arm kA", 1);

        if (lastControlMode != controlMode && controlMode == ControlMode.MANUAL) {
            setManualPower(0);
        }
        m_controller.setPID(kP, kI, kD);
        feedforward = new ArmFeedforward(kS, kG, kV, kA);
        velocities[calcVelocityIndex(velocityIndex)] = neoEncoder.getVelocity();
        switch (controlMode){
            case AUTOMATIC:
                super.periodic();
            case MANUAL:
                leftArmMotor.set(power);
                rightArmMotor.set(power);
                SmartDashboard.putNumber("right arm power", rightArmMotor.get());
                SmartDashboard.putNumber("left arm power", leftArmMotor.get());
        }
        ++velocityIndex;
        lastControlMode = controlMode;
    }
}