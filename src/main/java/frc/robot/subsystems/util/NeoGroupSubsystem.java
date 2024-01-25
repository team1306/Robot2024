package frc.robot.subsystems.util;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NeoGroupSubsystem extends SubsystemBase {
    private final List<Pair<CANSparkMax, Boolean>> otherNeos;
    private final CANSparkMax lead;

    @SafeVarargs
    public NeoGroupSubsystem(IdleMode idleMode, Consumer<CANSparkMax>[] motorIntializationFunctions, Pair<CANSparkMax, Boolean>... neos) {
        if (neos.length < 1) {
            throw new IllegalArgumentException("bro forgot to add motors 💀");
        }
        this.lead = neos[0].getFirst();
        lead.setInverted(true);
        this.otherNeos = Arrays.asList(neos);
        otherNeos.forEach(neoPair -> {
            final CANSparkMax neo = neoPair.getFirst();
            neo.setIdleMode(idleMode);
            if (motorIntializationFunctions != null) {
                for (Consumer<CANSparkMax> motorIntalizationFunction : motorIntializationFunctions) {
                    motorIntalizationFunction.accept(neo);
                }
            }
        });
        otherNeos.remove(0);

        for (Pair<CANSparkMax, Boolean> neoPair : otherNeos) {
            final boolean thisMotorInverted = neoPair.getSecond();
            neoPair.getFirst().follow(lead, neoPair.getSecond().booleanValue() ? !thisMotorInverted : thisMotorInverted);
        }
    }

    @SafeVarargs
    public NeoGroupSubsystem(IdleMode idleMode, Pair<CANSparkMax, Boolean>... neos) {
        this(idleMode, null, neos);
    }
    
    @SafeVarargs
    public NeoGroupSubsystem(Pair<CANSparkMax, Boolean>... neos) {
        this(IdleMode.kCoast, neos);
    }

    /**
     * Method that determines what power is going to go to the neos.
     * @return power to send to neo group
     */
    public abstract double getPowerOutput();

    @Override
    public void periodic() {
        final double power = getPowerOutput();
        for (Pair<CANSparkMax, Boolean> neoPair : otherNeos) {
            neoPair.getFirst().set(power);
        }
    }
}
