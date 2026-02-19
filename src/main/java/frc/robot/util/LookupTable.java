package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LookupTableConstants;

/** Handles lookup table functionality */
public class LookupTable implements Sendable {
    private double[] tableX, tableY;

    private int cachedInd = 0;

    private double calcError = 0.0;
    private double calcDist = 0.0;

    public LookupTable(double[][] inputTable, String tableName) {
        this.tableX = new double[inputTable.length];
        this.tableY = new double[inputTable.length];

        for (int i = 0; i < inputTable.length; i ++) {
            this.tableX[i] = inputTable[i][0];
            this.tableY[i] = inputTable[i][1];
        }

        SendableRegistry.add(this, tableName);
        SmartDashboard.putData(this);
    }

    /**
     * Gets the corresponding output from an input, uses linear interpolation
     * 
     * @param inputVal the input
     * @return the output
     */
    public double getOutput(double inputVal) {
        // Fast value return
        if (this.isInInd(this.cachedInd, inputVal)) {
            return this.linearInterpolate(this.cachedInd, inputVal);
        } else if (this.isInInd(this.cachedInd + 1, inputVal)) {
            this.cachedInd ++;
            return this.linearInterpolate(this.cachedInd, inputVal);
        } else if (this.isInInd(this.cachedInd - 1, inputVal)) {
            this.cachedInd --;
            return this.linearInterpolate(this.cachedInd, inputVal);
        }

        // If input is greater than the values the table has
        if (inputVal < this.tableX[0]) {
            return this.linearInterpolate(0, inputVal);
        } else if (inputVal > this.tableX[this.tableX.length - 1]) {
            return this.linearInterpolate(this.tableX.length - 2, inputVal);
        }

        // When the input is between two table values, do a linear interpolation
        for (int i = 0; i < this.tableX.length - 1; i ++) {
            if (this.isInInd(i, inputVal)) {
                this.cachedInd = i;
                // calculation
                return this.linearInterpolate(i, inputVal);
            }
        }

        return 0.0;
    }

    /**
     * Gets the slope at a point in the graph made by the table
     * 
     * @param inputVal the input
     * @return the slope at the input value
     */
    public double getOutputSlope(double inputVal) {
        // Fast value return
        if (this.isInInd(this.cachedInd, inputVal)) {
            return this.slopeAtInd(this.cachedInd);
        } else if (this.isInInd(this.cachedInd + 1, inputVal)) {
            this.cachedInd ++;
            return this.slopeAtInd(this.cachedInd);
        } else if (this.isInInd(this.cachedInd - 1, inputVal)) {
            this.cachedInd --;
            return this.slopeAtInd(this.cachedInd);
        }

        // If input is greater than the values the table has
        if (inputVal < this.tableX[0]) {
            return this.slopeAtInd(0);
        } else if (inputVal > this.tableX[this.tableX.length - 1]) {
            return this.slopeAtInd(this.tableX.length - 2);
        }

        // When the input is between two table values, do a linear interpolation
        for (int i = 0; i < this.tableX.length - 1; i ++) {
            if (this.isInInd(i, inputVal)) {
                this.cachedInd = i;
                // calculation
                return this.slopeAtInd(i);
            }
        }

        return 0.0;
    }

    /**
     * Calculation for Shoot On The Move using Newton's Method. Only use for the time-of-flight lookup table!
     * 
     * @param rV the robot (specifically turret) velocity, field-centric
     * @param dRH vector representing distance from turret to hub, starting at the turret
     * @return a Translation2d representing the vector of the distance and field-centric angle to use when calculating the shot
     */
    public Translation2d sotmCalc(Translation2d rV, Translation2d dRH) {
        double pointCalc = 0.0;
        double pointCalcPrev = 0.0;
        int startInd = this.tableX.length - 2;
        
        // Check which interval to calculate between
        for (int i = 0; i < this.tableX.length; i ++) {
            this.cachedInd = i;
            pointCalc = this.calcSingleDistMagnitude(this.tableX[i], rV, dRH);
            if (i > 0) {
                if (pointCalcPrev >= this.tableX[i - 1] && pointCalc <= this.tableX[i]) {
                    startInd = i - 1;
                    break;
                }
            }
            pointCalcPrev = pointCalc;
        }

        double denominator = rV.getNorm() * this.slopeAtInd(startInd) + 1;
        Translation2d finalVector = this.calcSingleDist(this.tableX[startInd], rV, dRH);

        for (int i = 0; i < LookupTableConstants.sotmCalcLoops; i ++) {
            Translation2d numerator = this.calcSingleDist(finalVector.getNorm(), rV, dRH).minus(finalVector);
            finalVector = finalVector.plus(numerator.div(denominator));
        }

        this.calcError = this.calcSingleDist(finalVector.getNorm(), rV, dRH).minus(finalVector).getNorm();
        return finalVector;
    }

    /**
     * Calculation for Shoot On The Move using the converging method. Only use for the time-of-flight lookup table!
     * 
     * @param rV the robot (specifically turret) velocity, field-centric
     * @param dRH vector representing distance from turret to hub, starting at the turret
     * @return a Translation2d representing the vector of the distance and field-centric angle to use when calculating the shot
     */
    public Translation2d sotmCalc2(Translation2d rV, Translation2d dRH) {
        Translation2d finalVector = new Translation2d(dRH.getX(), dRH.getY());

        for (int i = 0; i < LookupTableConstants.sotmCalcLoops; i ++) {
            finalVector = this.calcSingleDist(finalVector.getNorm(), rV, dRH);
        }

        this.calcError = this.calcSingleDist(finalVector.getNorm(), rV, dRH).minus(finalVector).getNorm();
        this.calcDist = finalVector.getNorm();
        return finalVector;
    }

    /**
     * Helps with calculation for Shoot On The Move
     * @param dist the distance
     * @param rV
     * @param dRH
     * @return the calculated distance vector
     */
    private Translation2d calcSingleDist(double dist, Translation2d rV, Translation2d dRH) {
        Translation2d velocityVector = rV.times(this.getOutput(dist));
        return dRH.minus(velocityVector);
    }

    /**
     * Helps with calculation for Shoot On The Move
     * @param dist the distance
     * @param rV
     * @param dRH
     * @return the magnitude of the calculated distance vector
     */
    private double calcSingleDistMagnitude(double dist, Translation2d rV, Translation2d dRH) {
        return this.calcSingleDist(dist, rV, dRH).getNorm();
    }

    /**
     * Checks if a value is within two entries of the table
     * @param ind the index of the starting entry
     * @param val the value to check
     * @return if tableX[ind] <= val <= tableX[ind + 1]
     */
    private boolean isInInd(int ind, double val) {
        if (ind >= this.tableX.length - 1 || ind < 0) return false;
        return this.tableX[ind] <= val && this.tableX[ind + 1] >= val;
    }

    /**
     * Returns the slope between to entries of the table
     * @param ind the index of the first entry
     * @return the slope between (tableX[ind], tableX[ind + 1])
     */
    private double slopeAtInd(int ind) {
        return (this.tableY[ind + 1] - this.tableY[ind]) / (this.tableX[ind + 1] - this.tableX[ind]);
    }

    /**
     * Does linear interpolation for a point at val between (tableX[ind], tableX[ind + 1])
     * @param ind the table entry
     * @param val the X value
     * @return linear interpolated value of a point at X = val
     */
    private double linearInterpolate(int ind, double val) {
        return this.tableY[ind] + (this.slopeAtInd(ind) * (val - this.tableX[ind]));
    }

    public double getCalcError() {
        return this.calcError;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty("Table X", () -> this.tableX, (newTableX) -> this.tableX = newTableX);
        builder.addDoubleArrayProperty("Table Y", () -> this.tableY, (newTableY) -> this.tableY = newTableY);
        builder.addDoubleProperty("Calculation Error", () -> this.getCalcError(), null);
        builder.addDoubleProperty("Calculation Distance", () -> this.calcDist, null);
    }
}
