package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.DashboardGetter;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends NeoGroupSubsystem{

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
        super.periodic();

        //TODO Likely don't use PID controller for direction, but use for speed
        //IDEA have difference of angle as input to pid controller
        double output = pidController.calculate(currentAngle.getDegrees(), targetAngle.getDegrees());

        SmartDashboard.putNumber("Turret Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("Turret Current Angle", currentAngle.getDegrees());
        SmartDashboard.putNumber("Turret PID Output", output);
        //TODO DECIDE ON BEST METHOD TO ROTATE TURRET
    }

    public Rotation2d getCurrentAngle(){
        //TODO GET CURRENT ANGLE OF TURRET
        return currentAngle;
    }

    public Rotation2d getTargetAngle(){
        return targetAngle;
    }

    public void setTargetAngle(Rotation2d targetAngle){
        this.targetAngle = targetAngle;
    }
}
