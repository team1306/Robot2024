package frc.robot.subsystems.util;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NeoGroupSubsystem extends SubsystemBase {
    private final List<Pair<CANSparkMax, Boolean>> otherNeos;
    private final CANSparkMax lead;

    public NeoGroupSubsystem(IdleMode idleMode, Pair<CANSparkMax, Boolean>... neos) {
        if (neos.length < 1) {
            throw new IllegalArgumentException("bro forgot to add motors 💀");
        }
        this.lead = neos[0].getFirst();
        lead.setInverted(true);
        this.otherNeos = Arrays.asList(neos);
        otherNeos.forEach(neoPair -> neoPair.getFirst().setIdleMode(idleMode));
        otherNeos.remove(0);

        for (Pair<CANSparkMax, Boolean> neoPair : otherNeos) {
            final boolean thisMotorInverted = neoPair.getSecond();
            neoPair.getFirst().follow(lead, neoPair.getSecond().booleanValue() ? !thisMotorInverted : thisMotorInverted);
        }
    }

    public abstract double getPowerOutput();

    @Override
    public void periodic() {
        final double power = getPowerOutput();
        for (Pair<CANSparkMax, Boolean> neoPair : otherNeos) {
            neoPair.getFirst().set(power);
        }
    }
}
