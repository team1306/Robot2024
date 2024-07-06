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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends NeoGroupSubsystem{

    //Maximum angle for turret. Will not allow ANY angle above this
    public final double MAX_TURRET_ANGLE_DEGREES = 900;
    public final Rotation2d MAX_TURRET_ANGLE = Rotation2d.fromDegrees(MAX_TURRET_ANGLE_DEGREES);

    private Rotation2d targetAngle = new Rotation2d();
    private Rotation2d currentAngle = new Rotation2d();

    private double kP = 1, kI = 0, kD = 0;
    private PIDController pidController = new PIDController(kP, kI, kD);

    public Turret() {
        super(new NeoData(MotorUtil.initSparkMax(TURRET_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake), false));
        super.relativeSpeed = 0;

        DashboardGetter.addGetDoubleData("Turret kP", kP, value -> kP = value);    
        DashboardGetter.addGetDoubleData("Turret kI", kI, value -> kI = value);        
        DashboardGetter.addGetDoubleData("Turret kD", kD, value -> kD = value);   
    }

    @Override
    public void periodic(){

        //TODO GET CURRENT ANGLE OF TURRET -> figure out if there will be an absolute encoder
        currentAngle = new Rotation2d();

        double output = pidController.calculate(currentAngle.getDegrees(), getAngleDifference().getDegrees());

        SmartDashboard.putNumber("Turret Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("Turret Current Angle", currentAngle.getDegrees());
        SmartDashboard.putNumber("Turret PID Output", output);

        //TODO DECIDE ON BEST METHOD TO ROTATE TURRET -> Maybe make into a command that other commands feed things into

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

    private Rotation2d clampTurretAngle(Rotation2d value){
        return Rotation2d.fromDegrees(MathUtil.clamp(value.getDegrees(), -MAX_TURRET_ANGLE_DEGREES, MAX_TURRET_ANGLE_DEGREES));
    }
}
