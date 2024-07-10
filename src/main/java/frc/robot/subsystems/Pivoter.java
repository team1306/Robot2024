package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.DashboardGetter;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pivoter extends NeoGroupSubsystem{ 
    private static final int LOW_ANGLE_BOUND = 0;
    private static final int HIGH_ANGLE_BOUND = 90;
    private static final double ENCODER_OFFSET = 0; 

    private Rotation2d targetAngle = new Rotation2d();
    private Rotation2d currentAngle = new Rotation2d();

    private double toleranceDegrees = 1;

    //TODO ADD A WAY TO GET ACCELERATION AND/OR VELOCITY??
    private double maxAcceleration = 1, maxVelocity = 1;

    private TrapezoidProfile.Constraints pivoterConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    private double kP = 1, kI = 0, kD = 0;
    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, pivoterConstraints, LOOP_TIME_SECONDS);

    private double kG = 0, kV = 0;
    private ArmFeedforward pivoterFeedforward = new ArmFeedforward(0, kG, kV);

    private DutyCycleEncoder leftEncoder = new DutyCycleEncoder(PIVOTER_LEFT_ENCODER_DIO_PORT);
    private DutyCycleEncoder rightEncoder = new DutyCycleEncoder(PIVOTER_RIGHT_ENCODER_DIO_PORT);

    public Pivoter(){
        super(
            new NeoData(MotorUtil.initSparkMax(PIVOTER_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake), false),
            new NeoData(MotorUtil.initSparkMax(PIVOTER_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake), true)
        );
        super.relativeSpeed = 0;

        DashboardGetter.addGetDoubleData("Pivoter kP", kP, value -> kP = value);    
        DashboardGetter.addGetDoubleData("Pivoter kI", kI, value -> kI = value);        
        DashboardGetter.addGetDoubleData("Pivoter kD", kD, value -> kD = value);        

        DashboardGetter.addGetDoubleData("Pivoter kG", kG, value -> kG = value);        
        DashboardGetter.addGetDoubleData("Pivoter kV", kV, value -> kV = value);   

        DashboardGetter.addGetDoubleData("Pivoter Max Velocity", maxVelocity, value -> maxVelocity = value);
        DashboardGetter.addGetDoubleData("Pivoter Max Acceleration", maxAcceleration, value -> maxAcceleration = value);

        DashboardGetter.addGetDoubleData("Pivoter Tolerance Degrees", toleranceDegrees, value -> toleranceDegrees = value);

        pidController.setTolerance(toleranceDegrees);

        setTargetAngle(internalGetCurrentAngle());
    }

    @Override
    public void periodic(){
        pivoterConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        pidController.setConstraints(pivoterConstraints);
        pidController.setPID(kP, kI, kD);

        pivoterFeedforward = new ArmFeedforward(0, kG, kV);        
        
        currentAngle = internalGetCurrentAngle();

        double pidOutput = pidController.calculate(currentAngle.getDegrees(), targetAngle.getDegrees());        
        if (!Utilities.isValidDouble(pidOutput)) pidOutput = 0;

        final State state = pidController.getSetpoint();

        double feedforwardOutput = pivoterFeedforward.calculate(Math.toRadians(state.position), Math.toRadians(state.velocity));
        if (!Utilities.isValidDouble(feedforwardOutput)) feedforwardOutput = 0;        

        super.relativeSpeed = MotorUtil.clampPercent(pidOutput + feedforwardOutput);

        SmartDashboard.putNumber("Pivoter Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("Pivoter Current Angle", currentAngle.getDegrees());
        SmartDashboard.putNumber("Pivoter PID Output", pidOutput);

        super.periodic();
    }

    public Rotation2d getCurrentAngle(){
        return currentAngle;
    }

    public Rotation2d getTargetAngle(){
        return targetAngle;
    }

    public void setTargetAngle(Rotation2d targetAngle){
        this.targetAngle = Rotation2d.fromDegrees(MathUtil.clamp(targetAngle.getDegrees(), LOW_ANGLE_BOUND, HIGH_ANGLE_BOUND));
    }

    public boolean atSetpoint(){
        return Math.abs(currentAngle.minus(targetAngle).getDegrees()) < toleranceDegrees;
    }

    private Rotation2d internalGetCurrentAngle(){
        return Rotation2d.fromRotations((leftEncoder.get() + rightEncoder.get()) / 2).minus(Rotation2d.fromDegrees(ENCODER_OFFSET));
    }
}
