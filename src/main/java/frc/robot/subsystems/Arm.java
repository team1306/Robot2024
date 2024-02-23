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
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.util.MotorUtil;

import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase  {
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
    private final Encoder relativeThroughBore;
    private final DutyCycleEncoder absoluteThroughBoreEncoder;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

    private final ProfiledPIDController profiledPIDController;
    private ArmFeedforward feedforward;

    public static double kP = 0.05, kI = 0, kD = 0; // Do we want PID Controller? Or do we want to do state space model?
                                                 // need to read https://file.tavsys.net/control/controls-engineering-in-frc.pdf more so I know what I am doing
    public static double kS = 0, kG = 0.0725, kA = 0, kV = .17;
    private static double kMaxVelocity = 360, kMaxAcceleration = 280; // kMA MIGHT BE WRONG

    public static final double INITIAL_POSITION = 0, DELTA_AT_SETPOINT = 0.01;

    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);
    private double power;

    private long velocityIndex = 0;
    private double[] velocities = new double[2];

    private ControlMode controlMode, lastControlMode;

    public Arm(ControlMode controlMode) {
        super("arm");
        leftArmMotor = MotorUtil.initSparkMax(ARM_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        rightArmMotor = MotorUtil.initSparkMax(ARM_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        leftArmMotor.setInverted(false);
        rightArmMotor.setInverted(true);
        
        absoluteThroughBoreEncoder = new DutyCycleEncoder(0);
        absoluteThroughBoreEncoder.reset();
        relativeThroughBore = new Encoder(2, 3, true, EncodingType.k1X);
        relativeThroughBore.reset();
        relativeThroughBore.setDistancePerPulse(360/2048); // DEGREES_PER_REVOLUTION / CYCLES PER REVOLUTION

    

        feedforward = new ArmFeedforward(kS, kG, kV, kA);
        profiledPIDController = new ProfiledPIDController(kP, kI, kD, m_constraints, LOOP_TIME_SECONDS);

        this.controlMode = controlMode;
        this.lastControlMode = controlMode;

        SmartDashboard.putNumber("Arm kP", .05);
        SmartDashboard.putNumber("Arm kI", 0);
        SmartDashboard.putNumber("Arm kD", 0);

        SmartDashboard.putNumber("Arm kS", 0);
        SmartDashboard.putNumber("Arm kG", 0);
        SmartDashboard.putNumber("Arm kV", 0);
        SmartDashboard.putNumber("Arm kA", 0);

        profiledPIDController.setTolerance(DELTA_AT_SETPOINT);
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
        return Rotation2d.fromRotations(absoluteThroughBoreEncoder.get() * -1);
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
        profiledPIDController.setPID(kP, kI, kD);
        feedforward = new ArmFeedforward(kS, kG, kV, kA);
        velocities[calcVelocityIndex(velocityIndex)] = relativeThroughBore.getRate();
        switch (controlMode){
            case AUTOMATIC, VISION:
                double pidOutput = profiledPIDController.calculate(getCurrentAngle().getDegrees(), getTargetAngle().getDegrees());
                if (Double.isNaN(pidOutput) || Double.isInfinite(pidOutput)) pidOutput = 0;
                SmartDashboard.putNumber("pid output", pidOutput);
                final State state = profiledPIDController.getSetpoint();
                double feedforwardOutput = feedforward.calculate(Math.toRadians(state.position), Math.toRadians(state.velocity));
                if (Double.isNaN(feedforwardOutput) || Double.isInfinite(feedforwardOutput)) feedforwardOutput = 0;
                SmartDashboard.putNumber("feedForward", feedforwardOutput);
                pidOutput += feedforwardOutput;
                SmartDashboard.putNumber("total arm output", pidOutput);

                leftArmMotor.set(MathUtil.clamp(pidOutput, -MoveArmCommand.peakOutput, MoveArmCommand.peakOutput));
                rightArmMotor.set(MathUtil.clamp(pidOutput, -MoveArmCommand.peakOutput, MoveArmCommand.peakOutput)); 
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
        return profiledPIDController.atGoal();
    }
}