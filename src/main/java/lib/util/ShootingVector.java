package lib.util;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ShootingVector {
    public Translation3d terminal;
    public Translation3d initial;

    public ShootingVector(Translation3d terminal, Translation3d inital) {
        this.initial = inital;
        this.terminal = terminal;
    }

    public ShootingVector(double xt, double yt, double zt, double xi, double yi, double zi) {
        this.initial = new Translation3d(xi, yi, zi);
        this.terminal = new Translation3d(xt, yt, zt);
    }

    public double getDistance() {
        return initial.getDistance(terminal);
    }

    public Translation3d interpolate() {
        return initial.interpolate(terminal, 1);
    }

    public Translation3d interpolate(Translation3d translation) {
        return initial.interpolate(translation, 1);
    }

    public void addTerminal(Translation3d addition) {
        terminal.plus(addition);
    }

    public void minusTerminal(Translation3d subtraction) {
        terminal.minus(subtraction);
    }

    public void addInitial(Translation3d addition) {
        initial.plus(addition);
    }

    public void minusInitial(Translation3d subtraction) {
        initial.minus(subtraction);
    }

    public void addShootingVector(ShootingVector vector) {
        addInitial(vector.initial);
        addTerminal(vector.terminal);
    }

    public void minusShootingVector(ShootingVector vector) {
        minusInitial(vector.initial);
        minusTerminal(vector.terminal);
    }
}
