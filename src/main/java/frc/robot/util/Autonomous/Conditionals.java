package frc.robot.util.Autonomous;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public interface Conditionals {
    public BooleanSupplier intakeCovered = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return Intake.getInstance().sensorCovered();
        }
    };

    public BooleanSupplier shooterLoaded = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return Shooter.getInstance().loaded();
        }
    };

    public BooleanSupplier shooterStowed = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return Shooter.getInstance().atStow();
        }
    };

    public BooleanSupplier hasNote = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return Shooter.getInstance().loaded() || Intake.getInstance().sensorCovered();
        }
    };
}
