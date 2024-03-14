package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;

import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase  {
    public enum ControlMode {
        MANUAL,
        AUTOMATIC,
        VISION
    }


    public interface Setpoint {
        double getPos();

        static class Custom implements Setpoint {
            private final double pos;

            public Custom(double pos) {
                this.pos = pos;
            }

            public Custom(Rotation2d pos) {
                this(pos.getDegrees());
            }
            
            @Override
            public double getPos() {
                return pos;
            }
        }
    }

    public enum SetpointOptions implements Setpoint {
        AMP(108),
        INTAKE(0),
        DOWN(4),
        SHOOT_CLOSE(16),
        STAGE_SHOT(35);

        private final double pos;

        private SetpointOptions(double pos) {
            this.pos = pos;
        }

        @Override
        public double getPos() {
            return pos;
        }
    }

    private final CANSparkMax leftArmMotor;
    private final CANSparkMax rightArmMotor;
    private final Encoder relativeThroughBore;
    private final DutyCycleEncoder absoluteThroughBoreEncoder;

    private TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

    private final ProfiledPIDController profiledPIDController;
    private ArmFeedforward feedforward;

    public static double kP = 0.02, kI = 0.0005, kD = 0.0018; // Do we want PID Controller? Or do we want to do state space model?
                                                 // need to read https://file.tavsys.net/control/controls-engineering-in-frc.pdf more so I know what I am doing
    public static double kG = 0.0725, kV = .17;
    private static double kMaxVelocity = 360, kMaxAcceleration = 140; // kMA MIGHT BE WRONG
    private double armMaxPower = 1;

    public static final double OFFSET = -219.15 + 180 + 10 + .5 + 57.15 + 174.425, DELTA_AT_SETPOINT = 1;
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);
    private double manualPower;

    private long velocityIndex = 0;
    private double[] velocities = new double[2];

    private ControlMode controlMode, lastControlMode;

    public Arm(ControlMode controlMode) {
        super("arm");
        leftArmMotor = MotorUtil.initSparkMax(ARM_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        rightArmMotor = MotorUtil.initSparkMax(ARM_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        leftArmMotor.setInverted(true);
        rightArmMotor.setInverted(false);
        
        absoluteThroughBoreEncoder = new DutyCycleEncoder(0);
        relativeThroughBore = new Encoder(2, 3, true, EncodingType.k1X);
        relativeThroughBore.reset();
        relativeThroughBore.setDistancePerPulse(360D/2048D); // DEGREES_PER_REVOLUTION / CYCLES PER REVOLUTION

        feedforward = new ArmFeedforward(0, Math.min(0.3, kG), kV, 0);
        profiledPIDController = new ProfiledPIDController(kP, kI, kD, m_constraints, LOOP_TIME_SECONDS);

        this.controlMode = controlMode;
        this.lastControlMode = controlMode;

        SmartDashboard.putNumber("Arm kP", kP);
        SmartDashboard.putNumber("Arm kI", kI);
        SmartDashboard.putNumber("Arm kD", kD);

        SmartDashboard.putNumber("Arm kG", kG);
        SmartDashboard.putNumber("Arm kV", kV);
        SmartDashboard.putNumber("Arm kMaxAcceleration", kMaxAcceleration);
        SmartDashboard.putNumber("Arm Peak Output", armMaxPower);

        profiledPIDController.setTolerance(DELTA_AT_SETPOINT);
        setTargetAngle(getCurrentAngle());
    }
    
    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    public Arm() {
        this(ControlMode.AUTOMATIC);
    }
        
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations((absoluteThroughBoreEncoder.get() * -1)).minus(Rotation2d.fromDegrees(OFFSET));
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


    /**
     * Gets measurement of PID system
     * 
     * @return output of current arm rotation in degrees
     * @see PIDSubsystem#getTargetAngle()
     */
    protected double getMeasurement() {
       return getCurrentAngle().getDegrees();
    }

    public void setManualPower(double power) {
        this.manualPower = power;
    }

    @Override
    public void periodic() {
        kP = SmartDashboard.getNumber("Arm kP", kP);
        kI = SmartDashboard.getNumber("Arm kI", kI);
        kD = SmartDashboard.getNumber("Arm kD",kD);

        kG = SmartDashboard.getNumber("Arm kG", kG);
        kV = SmartDashboard.getNumber("Arm kV", kV);

        kMaxAcceleration = SmartDashboard.getNumber("Arm kMaxAcceleration", 0);
        armMaxPower = SmartDashboard.getNumber("Arm Peak Output", 1);
        m_constraints = new TrapezoidProfile.Constraints(m_constraints.maxVelocity, kMaxAcceleration);
        profiledPIDController.setConstraints(m_constraints);
        profiledPIDController.setPID(kP, kI, kD);
        feedforward = new ArmFeedforward(0, Math.min(0.25,kG), kV, 0);

        velocities[calcVelocityIndex(velocityIndex)] = relativeThroughBore.getRate();
        final double motorPower = MathUtil.applyDeadband(MathUtil.clamp(switch (controlMode) {
            case AUTOMATIC, VISION -> {
                double pidOutput = profiledPIDController.calculate(getCurrentAngle().getDegrees(), getTargetAngle().getDegrees());
                if (Double.isNaN(pidOutput) || Double.isInfinite(pidOutput)) pidOutput = 0;
                SmartDashboard.putNumber("pid output", pidOutput);
                final State state = profiledPIDController.getSetpoint();
                double feedforwardOutput = feedforward.calculate(Math.toRadians(state.position), Math.toRadians(state.velocity));
                if (Double.isNaN(feedforwardOutput) || Double.isInfinite(feedforwardOutput)) feedforwardOutput = 0;
                SmartDashboard.putNumber("feedForward", feedforwardOutput);
                pidOutput += feedforwardOutput;
                SmartDashboard.putNumber("total arm output", pidOutput);
                SmartDashboard.putNumber("arm acceleration", calculateAcceleration());
                yield (getCurrentAngle().getDegrees() < 2.5 && getTargetAngle().getDegrees() < 2.5 ? 0 : pidOutput);
            }
            case MANUAL -> {
                if (lastControlMode != ControlMode.MANUAL) {
                    setManualPower(0);
                    yield 0;
                }
                yield manualPower;
            }
        }, -armMaxPower, armMaxPower), .005);

        leftArmMotor.set(motorPower);
        rightArmMotor.set(motorPower);

        SmartDashboard.putNumber("right arm power", rightArmMotor.get());
        SmartDashboard.putNumber("left arm power", leftArmMotor.get());
        SmartDashboard.putNumber("arm current", rightArmMotor.getOutputCurrent() + leftArmMotor.getOutputCurrent());
        SmartDashboard.putNumber("Arm Current Angle", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm Raw Velocity", relativeThroughBore.getRate());
        
        lastControlMode = controlMode;
        ++velocityIndex;
    }

    public boolean atSetpoint() {
        return profiledPIDController.atGoal();
    }
}