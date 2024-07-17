package frc.robot.subsystems.utils;

import edu.wpi.first.math.geometry.Rotation2d;

@FunctionalInterface
public interface PivoterSetpoint {
    Rotation2d getAngle();

    public static class Custom implements PivoterSetpoint{
        private final Rotation2d angle;
        public Custom(Rotation2d angle){
            this.angle = angle;
        }

        @Override
        public Rotation2d getAngle() {
            return angle;
        }
    }

    public enum Options implements PivoterSetpoint{
            DOWN(0);
    
            private final Rotation2d angle;
    
            Options(Rotation2d angle) {
                this.angle = angle;
            }

            Options(double angleDegrees) {
                this(Rotation2d.fromDegrees(angleDegrees));
            }
    
            @Override
            public Rotation2d getAngle() {
                return angle;
            }
    }
}
