package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.DashboardGetter;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pivoter extends NeoGroupSubsystem{ 
    private static final int LOW_ANGLE_BOUND = 0;
    private static final int HIGH_ANGLE_BOUND = 90;
    private static final double ENCODER_OFFSET = 0; 

    private Rotation2d targetAngle = new Rotation2d();

    private double kP = 1, kI = 0, kD = 0;
    private PIDController pidController = new PIDController(kP, kI, kD);

    //TODO ENCODER INIT AND TUNING
    private DutyCycleEncoder leftEncoder;
    private DutyCycleEncoder rightEncoder;

    public Pivoter(){
        super(
            new NeoData(MotorUtil.initSparkMax(PIVOTER_LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake), false),
            new NeoData(MotorUtil.initSparkMax(PIVOTER_RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake), true)
        );
        super.relativeSpeed = 0;

        DashboardGetter.addGetDoubleData("Pivoter kP", kP, value -> kP = value);    
        DashboardGetter.addGetDoubleData("Pivoter kI", kI, value -> kI = value);        
        DashboardGetter.addGetDoubleData("Pivoter kD", kD, value -> kD = value);        
    
    }

    @Override
    public void periodic(){
        super.periodic();

        double output = pidController.calculate(getCurrentAngle().getDegrees(), targetAngle.getDegrees());
        
        SmartDashboard.putNumber("Pivoter Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("Pivoter Current Angle", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Pivoter PID Output", output);
    }

    public Rotation2d getCurrentAngle(){
        return Rotation2d.fromRotations((leftEncoder.get() + rightEncoder.get()) / 2).minus(Rotation2d.fromDegrees(ENCODER_OFFSET));
    }

    public Rotation2d getTargetAngle(){
        return targetAngle;
    }

    public void setTargetAngle(Rotation2d targetAngle){
        this.targetAngle = Rotation2d.fromDegrees(MathUtil.clamp(targetAngle.getDegrees(), LOW_ANGLE_BOUND, HIGH_ANGLE_BOUND));
    }
}
