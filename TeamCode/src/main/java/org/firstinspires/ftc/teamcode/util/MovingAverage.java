package org.firstinspires.ftc.teamcode.util;

/**
 * Created by smvoigt on 6/12/2017.
 */

public class MovingAverage {

    // A ring buffer is used to keep track of a moving average
    private final int ringBufferSize;
    private final double ringBuffer[];
    private int ringBufferIndex = 0;

    private double movingTotal;
    private double movingAverage;
    private double runningTotal;
    private long count;

    public MovingAverage(int num) {
        ringBufferSize = num;
        ringBuffer = new double[ringBufferSize];
        reset();
    }

    public void reset() {
        count = 0;
        movingTotal = 0.0;
        movingAverage = 0.0;
        runningTotal = 0.0;
        for (int i=0; i<ringBufferSize; i++) {
            ringBuffer[i]=0;
        }
    }

    public void add(double value) {
        // Adjust the running total
        movingTotal = movingTotal - ringBuffer[ringBufferIndex] + value;
        runningTotal = runningTotal + value;

        // increment the count
        count += 1;

        // Add the new value
        ringBuffer[ringBufferIndex] = value;

        // update the moving average value
        if (count < ringBufferSize) {
            movingAverage = movingTotal / (double) count;
        }
        else {
            movingAverage = movingTotal / (double) ringBufferSize;
        }

        // wrap the current index
        ringBufferIndex = (ringBufferIndex + 1) % ringBufferSize;
    }

    public double getMovingAverage() {
        return movingAverage;
    }

}
