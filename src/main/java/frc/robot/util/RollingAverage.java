package frc.robot.util;

public class RollingAverage {
    private double[] nums;
    private int len;
    private double average;

    public RollingAverage() {
        nums = new double[5];
        len = 5;
        average = 0;
    }

    public RollingAverage(int len) {
        nums = new double[len];
        this.len = len;
        average = 0;
    }

    public double updateValue(double nextValue) {
        double sum = nextValue;
        for (int i = 0; i < len - 1; i++) {
            nums[i] = nums[i + 1];
            sum += nums[i];
        }
        nums[len - 1] = nextValue;
        average = sum / len;
        return average;
    }

    public double getLast() {
        return nums[len - 1];
    }

    public int getLen() {
        return len;
    }

    public double[] getNums() {
        return nums.clone();
    }

    public void setLength(int len) {
        double[] newNums = new double[len];
        for (int i = 0; i < len; i++) {
            if (i >= this.len) {
                newNums[i] = nums[this.len - i];
            }
        }
        nums = newNums;
    }

    public double getAverage() {
        return average;
    }
}