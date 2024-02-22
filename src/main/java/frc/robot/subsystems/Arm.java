package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.util.MotorUtil;

import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.*;

public class Arm extends ProfiledPIDSubsystem  {
    public enum ControlMode {
        MANUAL,
        AUTOMATIC,
        VISION
    }

    public enum Setpoint {
        AMP(100),
        INTAKE(0),
        SHOOT_CLOSE(5),
        STAGE_SHOT(10);

        public final int pos;

        private Setpoint(int pos) {
            this.pos = pos;
        }
    }

    private final CANSparkMax leftArmMotor;
    private final CANSparkMax rightArmMotor;
    private final DutyCycleEncoder throughBoreEncoder;
    
    private final RelativeEncoder neoEncoder;

    private ArmFeedforward feedforward;

    public static double kP = .05, kI = 0, kD = 0; // Do we want PID Controller? Or do we want to do state space model?
                                                 // need to read https://file.tavsys.net/control/controls-engineering-in-frc.pdf more so I know what I am doing
    public static double kS = 0, kG = 0, kA = 0, kV = 0; // PLACEHOLDER

    public static final double INITIAL_POSITION = 0, DELTA_AT_SETPOINT = 0.01;

    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);
    private double power;

    private long velocityIndex = 0;
    private double[] velocities = new double[2];

    private ControlMode controlMode, lastControlMode;

    public Arm(ControlMode controlMode) {
        super(new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(360, 720)), INITIAL_POSITION);
        
        leftArmMotor = MotorUtil.initSparkMax(ARM_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        rightArmMotor = MotorUtil.initSparkMax(ARM_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        leftArmMotor.setInverted(false);
        rightArmMotor.setInverted(true);
        
        throughBoreEncoder = new DutyCycleEncoder(0);
        throughBoreEncoder.reset();

        neoEncoder = rightArmMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, NEO_COUNTS_PER_REVOLUTION);
        neoEncoder.setPosition(0);
        neoEncoder.setPositionConversionFactor(Units.rotationsToRadians(1)); // REVOLUTIONS -> RAD
        neoEncoder.setVelocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1/100)); // RPM -> RAD/SEC
    

        feedforward = new ArmFeedforward(kS, kG, kV, kA);
    
        this.controlMode = controlMode;
        this.lastControlMode = controlMode;

        SmartDashboard.putNumber("Arm kP", .05);
        SmartDashboard.putNumber("Arm kI", 0);
        SmartDashboard.putNumber("Arm kD", 0);

        SmartDashboard.putNumber("Arm kS", 0);
        SmartDashboard.putNumber("Arm kG", 0);
        SmartDashboard.putNumber("Arm kV", 0);
        SmartDashboard.putNumber("Arm kA", 0);

        m_controller.setTolerance(DELTA_AT_SETPOINT);

    }
    
    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    public Arm() {
        this(ControlMode.MANUAL);
        m_enabled = true;
    }
        
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(throughBoreEncoder.get() * -1);
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
    protected void useOutput(double output, State state) {
        if (Double.isNaN(output) || Double.isInfinite(output)) output = 0;
        final double feedforwardVal = feedforward.calculate(state.position, state.velocity);
        output += feedforwardVal;
        SmartDashboard.putNumber("feedForward", feedforwardVal);
        leftArmMotor.set(MathUtil.clamp(output, -MoveArmCommand.peakOutput, MoveArmCommand.peakOutput));
        rightArmMotor.set(MathUtil.clamp(output, -MoveArmCommand.peakOutput, MoveArmCommand.peakOutput)); 
    }

    /**
     * Gets measurement of PID system
     * 
     * @return output of current arm rotation in degrees
     * @see PIDSubsystem#getTargetAngle()
     */
    @Override
    protected double getMeasurement() {
       return getCurrentAngle().getDegrees();
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
            case AUTOMATIC, VISION:
                m_controller.setGoal(getTargetAngle().getDegrees());
                super.periodic();
                break;
            case MANUAL:
                leftArmMotor.set(power);
                rightArmMotor.set(power);
                break;
        }
        SmartDashboard.putNumber("right arm power", rightArmMotor.get());
        SmartDashboard.putNumber("left arm power", leftArmMotor.get());
        ++velocityIndex;
        SmartDashboard.putNumber("arm current", rightArmMotor.getOutputCurrent() + leftArmMotor.getOutputCurrent());
        lastControlMode = controlMode;
    }

    public boolean atSetpoint() {
        return m_controller.atGoal();
    }
}