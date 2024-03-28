package frc.robot.util;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.NEO_CURRENT_LIMIT_AMPS;

public class MotorUtil {
    public static CANSparkMax initSparkMax(int motorId, MotorType motorType, IdleMode idleMode, int currentLimitAmps) {
        CANSparkMax motor = new CANSparkMax(motorId, motorType);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(idleMode);
        motor.setSmartCurrentLimit(currentLimitAmps);
        motor.burnFlash();
        return motor;
    }

    /**
     * Create a new CANSparkMax Neo motor
     * @param motorId the CAN id of the motor
     * @param motorType the type of the motor (Brushed or Brushless)
     * @param idleMode the idle mode of the motor (Brake or Coast)
     * @return the motor with the paramenters specified
     */
    public static CANSparkMax initSparkMax(int motorId, MotorType motorType, IdleMode idleMode) {
        return initSparkMax(motorId, motorType, idleMode, NEO_CURRENT_LIMIT_AMPS);
    }

    /**
     * Clamp the percent output to be between -1 and 1  
     * @return the clamped result
     */
    public static double clampPercent(double percent){
        return MathUtil.clamp(percent, -1, 1);
    }
    
}
