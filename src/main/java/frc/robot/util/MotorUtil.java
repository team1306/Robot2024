package frc.robot.util;

import static frc.robot.Constants.NEO_CURRENT_LIMIT_AMPS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class MotorUtil {
    /**
     * Create a new CANSparkMax Neo motor
     * @param motorId the CAN id of the motor
     * @param motorType the type of the motor (Brushed or Brushless)
     * @param idleMode the idle mode of the motor (Brake or Coast)
     * @return the motor with the paramenters specified
     */

    public static CANSparkMax initSparkMax(int motorId, MotorType motorType, IdleMode idleMode ){
        CANSparkMax motor = new CANSparkMax(motorId, motorType);
        motor.setIdleMode(idleMode);
        motor.setSmartCurrentLimit(NEO_CURRENT_LIMIT_AMPS);
        return motor;
    }

    /**
     * Clamp the percent output to be between -1 and 1  
     * @return the clamped result
     */
    public static double clampPercent(double percent){
        return MathUtil.clamp(percent, -1, 1);
    }
    
}
