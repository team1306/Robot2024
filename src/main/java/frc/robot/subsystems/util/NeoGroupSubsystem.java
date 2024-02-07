package frc.robot.subsystems.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Deprecated // NEEDS TO BE FIXED FOR BUGS
public abstract class NeoGroupSubsystem extends SubsystemBase {
    private final List<Pair<CANSparkMax, Boolean>> otherNeos;
    private final CANSparkMax lead;

    @SafeVarargs
    public NeoGroupSubsystem(Consumer<CANSparkMax>[] motorIntializationFunctions, Pair<CANSparkMax, Boolean>... neos) {
        if (neos.length < 1) {
            throw new IllegalArgumentException("bro forgot to add motors 💀");
        }
        this.lead = neos[0].getFirst();
        final boolean leadInverted = neos[0].getSecond();
        lead.setInverted(leadInverted);
        this.otherNeos = new ArrayList<>(Arrays.asList(neos));
        if (motorIntializationFunctions != null) {
            otherNeos.forEach(neoPair -> {
                final CANSparkMax neo = neoPair.getFirst();
                for (Consumer<CANSparkMax> motorIntalizationFunction : motorIntializationFunctions) {
                    motorIntalizationFunction.accept(neo);
                }
            });
        }
        otherNeos.remove(0);
        for (Pair<CANSparkMax, Boolean> neoPair : otherNeos) {
            neoPair.getFirst().follow(lead, neoPair.getSecond().booleanValue() == leadInverted ? false : true);
        }
    }
    
    @SafeVarargs
    public NeoGroupSubsystem(Pair<CANSparkMax, Boolean>... neos) {
        this(null, neos);
    }

    /**
     * Method that determines what power is going to go to the neos.
     * @return power to send to neo group
     */
    public abstract double getPowerOutput();

    @Override
    public void periodic() {
        final double power = getPowerOutput();
        otherNeos.forEach(pair -> pair.getFirst().set(power));
    }
}
