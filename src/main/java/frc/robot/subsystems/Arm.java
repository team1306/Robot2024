package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.util.DashboardGetter;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase  {

    private static double a = -0.0009, b = 0.366, c = 3.48;
    //-0.000905, 0.368, 3.41
    public interface Setpoint {
        double getPos();

        class Custom implements Setpoint {
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

        SetpointOptions(double pos) {
            this.pos = pos;
        }

        @Override
        public double getPos() {
            return pos;
        }
    }

    private final CANSparkMax leftArmMotor, rightArmMotor;
    private final Encoder relativeThroughBore;
    private final DutyCycleEncoder absoluteThroughBoreEncoder;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, maxAcceleration);

    private final ProfiledPIDController profiledPIDController;
    private ArmFeedforward feedforward;

    public static double kP = 0.02, kI = 0.0005, kD = 0.0018; // Do we want PID Controller? Or do we want to do state space model?
                                                 // need to read https://file.tavsys.net/control/controls-engineering-in-frc.pdf more so I know what I am doing
    public static double kG = 0.0725, kV = .17;
    private static final double MAX_VELOCITY = 360;
    private static double maxAcceleration = 140; // kMA MIGHT BE WRONG
    private double armMaxPower = 1;

    public static final double OFFSET = -219.15 + 180 + 10 + .5 + 57.15 + 174.425, DELTA_AT_SETPOINT = .3;
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    private long velocityIndex = 0;
    private final double[] velocities = new double[2];


    public Arm() {
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
        profiledPIDController = new ProfiledPIDController(kP, kI, kD, constraints, LOOP_TIME_SECONDS);

        DashboardGetter.addGetDoubleData("Arm kP", kP, value -> kP = value);
        DashboardGetter.addGetDoubleData("Arm kI", kI, value -> kI = value);
        DashboardGetter.addGetDoubleData("Arm kD", kD, value -> kD = value);

        DashboardGetter.addGetDoubleData("Arm kG", kG, value -> kG = value);
        DashboardGetter.addGetDoubleData("Arm kV", kV, value -> kV = value);

        DashboardGetter.addGetDoubleData("Arm MAX_ACCELERATION", maxAcceleration, value -> maxAcceleration = value);
        DashboardGetter.addGetDoubleData("Arm Peak Output", armMaxPower, value -> armMaxPower = value);

        DashboardGetter.addGetDoubleData("Arm Pitch A", a, value -> a = value);
        DashboardGetter.addGetDoubleData("Arm Pitch B", b, value -> b = value);
        DashboardGetter.addGetDoubleData("Arm Pitch C", c, value -> c = value); 

        profiledPIDController.setTolerance(DELTA_AT_SETPOINT);
        setTargetAngle(getCurrentAngle());
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

    @Override
    public void periodic() {
        constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAcceleration);
        profiledPIDController.setConstraints(constraints);
        profiledPIDController.setPID(kP, kI, kD);
        feedforward = new ArmFeedforward(0, Math.min(0.25,kG), kV, 0);

        velocities[calcVelocityIndex(velocityIndex)] = relativeThroughBore.getRate();

        double pidOutput = profiledPIDController.calculate(getCurrentAngle().getDegrees(), getTargetAngle().getDegrees());
        if (Double.isNaN(pidOutput) || Double.isInfinite(pidOutput)) pidOutput = 0;

        final State state = profiledPIDController.getSetpoint();
        double feedforwardOutput = feedforward.calculate(Math.toRadians(state.position), Math.toRadians(state.velocity));
        if (Double.isNaN(feedforwardOutput) || Double.isInfinite(feedforwardOutput)) feedforwardOutput = 0;

        pidOutput += feedforwardOutput;


        final double motorPower =
                MathUtil.applyDeadband(
                    MathUtil.clamp(getCurrentAngle().getDegrees() < 2.5 && getTargetAngle().getDegrees() < 2.5 ? 0 : pidOutput, -armMaxPower, armMaxPower),
                        .005);

        leftArmMotor.set(motorPower);
        rightArmMotor.set(motorPower);

        SmartDashboard.putNumber("feedForward", feedforwardOutput);
        SmartDashboard.putNumber("pid output", pidOutput);
        SmartDashboard.putNumber("total arm output", pidOutput);
        SmartDashboard.putNumber("arm acceleration", calculateAcceleration());
        SmartDashboard.putNumber("right arm power", rightArmMotor.get());
        SmartDashboard.putNumber("left arm power", leftArmMotor.get());
        SmartDashboard.putNumber("arm current", rightArmMotor.getOutputCurrent() + leftArmMotor.getOutputCurrent());

        SmartDashboard.putNumber("Arm Current Angle", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm Raw Velocity", relativeThroughBore.getRate());
        
        ++velocityIndex;
    }

    public InstantCommand getPitchControlCommand(DriveTrain driveTrain){
        return new InstantCommand(() -> {
            //38.0625 is the distance from the subwoofer to the speaker and tape in inches
            //2.3125 is the distance from the center of the limelight to the edge
            double speakerDistance = Utilities.getSpeakerDistance(driveTrain.getPose());
            
            //Theta must be in terms of degrees
            double theta = a * Math.pow(speakerDistance, 2) + b * speakerDistance + c;
            theta = MathUtil.clamp(theta, 0, 90);
            SmartDashboard.putNumber("Arm auto pitch angle", theta);
            // Set the target angle of the arm
            new MoveArmToSetpointCommand(this, new Setpoint.Custom(theta)).schedule();
        }  , this);
    }
}