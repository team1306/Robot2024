package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.DashboardGetter;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;

import static frc.robot.Constants.*;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends NeoGroupSubsystem{

    //Maximum angle for turret. Will not allow ANY angle above this
    public static final double MAX_TURRET_ANGLE_DEGREES = 850;
    public static final Rotation2d MAX_TURRET_ANGLE = Rotation2d.fromDegrees(MAX_TURRET_ANGLE_DEGREES);

    private static final double TURRET_GEAR_RATIO = 5;

    private Rotation2d targetAngle = new Rotation2d();
    private Rotation2d currentAngle = new Rotation2d();

    private double kP = 1, kI = 0, kD = 0;
    private PIDController pidController = new PIDController(kP, kI, kD);
    
    private double kS = 0, kV = 0;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);
    
    private Encoder absoluteEncoder = new Encoder(TURRET_ENCODER_DIO_PORT_A, TURRET_ENCODER_DIO_PORT_B, false, EncodingType.k1X);

    public Turret() {
        super(new NeoData(MotorUtil.initSparkMax(TURRET_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake), false));
        super.relativeSpeed = 0;

        absoluteEncoder.setDistancePerPulse(1 / 4096D / TURRET_GEAR_RATIO);
        absoluteEncoder.reset();

        DashboardGetter.addGetDoubleData("Turret kP", kP, value -> kP = value);    
        DashboardGetter.addGetDoubleData("Turret kI", kI, value -> kI = value);        
        DashboardGetter.addGetDoubleData("Turret kD", kD, value -> kD = value); 

        DashboardGetter.addGetDoubleData("Turret kS", kS, value -> kS = value); 
        DashboardGetter.addGetDoubleData("Turret kV", kV, value -> kV = value); 

        DashboardGetter.addGetBooleanData("Reset Turret Angle", false, value -> {
            if(value){ 
                absoluteEncoder.reset();
                SmartDashboard.putBoolean("Reset Turret Angle", false);
            }
        });
    }

    @Override
    public void periodic(){
        pidController.setPID(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        currentAngle = Rotation2d.fromDegrees(absoluteEncoder.getDistance());

        double pidOutput = pidController.calculate(currentAngle.getDegrees(), getAngleDifference().getDegrees());
        if(!Utilities.isValidDouble(pidOutput)) pidOutput = 0;

        double feedforwardOutput = feedforward.calculate(absoluteEncoder.getRate());
        if(!Utilities.isValidDouble(feedforwardOutput)) feedforwardOutput = 0;

        super.relativeSpeed = pidOutput + feedforwardOutput;

        SmartDashboard.putNumber("Turret Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("Turret Current Angle", currentAngle.getDegrees());
        SmartDashboard.putNumber("Turret PID Output", pidOutput);
        SmartDashboard.putNumber("Turret Feedforward Output", feedforwardOutput);

        super.periodic();

    }

    public Rotation2d getCurrentAngle(){
        return currentAngle;
    }

    public Rotation2d getAngleDifference(){
        return targetAngle.minus(currentAngle);
    }

    public Rotation2d getTargetAngle(){
        return targetAngle;
    }

    public void setAngleDifference(Rotation2d angleDifference){
        setTargetAngle(currentAngle.plus(angleDifference));
    }

    public void setTargetAngle(Rotation2d targetAngle){
        this.targetAngle = clampTurretAngle(targetAngle);
    }

    private static Rotation2d clampTurretAngle(Rotation2d value){
        return Rotation2d.fromDegrees(MathUtil.clamp(value.getDegrees(), -MAX_TURRET_ANGLE_DEGREES, MAX_TURRET_ANGLE_DEGREES));
    }


    //See https://www.desmos.com/calculator/4ijjqb4nci for a graph of the score and cost functions
    public static double scoreFunction(double position, double distance){
        return 100 * costFunction(position) / distance;
    }

    public static double costFunction(double position){
        double value = -Math.pow(1 / 550D * position, 6) + 10;
        return Math.max(value, 0);
    }
}
