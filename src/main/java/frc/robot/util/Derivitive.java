package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Calculates the derivitive using a rolling average
 */
public class Derivitive {
    private RollingAverage avg;
    
    private double lastVal;
    private double lastTime;

    public Derivitive() {
        avg = new RollingAverage();

        lastVal = Integer.MIN_VALUE;
        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Updates the derivitive
     * @param val value to add into average
     */
    public void update(double val) {
        if(lastVal != Integer.MIN_VALUE)
            avg.updateValue((val - lastVal) / (Timer.getFPGATimestamp() - lastTime));

        lastVal = val;
        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Gets derivitive in value/sec
     * @return Returns the derivitive
     */
    public double get() {
        return avg.getAverage();
    }
}
