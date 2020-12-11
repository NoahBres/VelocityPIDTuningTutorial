package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

public class VelocityPIDFController {
    private static final double ACCEL_TIMEOUT = 0.1;
    private static final double EPSILON = 1e-6;

    private PIDFController controller;

    private ElapsedTime accelTimeoutTimer;

    private MovingStatistics accelSamples;

    private double lastPosition;
    private double lastVelocity;

    private VelocityPIDFController() {
    }

    public VelocityPIDFController(PIDCoefficients pid) {
        this(pid, 0.0, 0.0, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV) {
        this(pid, kV, 0.0, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV, double kA) {
        this(pid, kV, kA, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV, double kA, double kStatic) {
        controller = new PIDFController(pid, kV, kA, kStatic);
        accelTimeoutTimer = new ElapsedTime();
        accelSamples = new MovingStatistics(5);
    }

    public void setTargetAcceleration(double acceleration) {
        controller.setTargetAcceleration(acceleration);
    }

    public void setTargetVelocity(double velocity) {
        controller.setTargetVelocity(velocity);
        controller.setTargetPosition(velocity);
    }

    private double calculateAccel(double measuredPosition, double measuredVelocity) {
        double dx = measuredPosition - lastPosition;
        if (dx != 0.0) {
            double accel = (measuredVelocity * measuredVelocity - lastVelocity * lastVelocity) / (2.0 * dx);

            // This is to avoid reporting a zero acceleration when the velocity does not change due to the internal
            // velocity measurement loop potentially running less frequently than our control loop
            if (Math.abs(measuredVelocity - lastVelocity) > EPSILON || accelTimeoutTimer.seconds() > ACCEL_TIMEOUT) {
                lastPosition = measuredPosition;
                lastVelocity = measuredVelocity;

                accelSamples.add(accel);
                accelTimeoutTimer.reset();
            }
            return accelSamples.getMean();
        } else {
            return 0.0;
        }
    }

    public double update(double measuredPosition, double measuredVelocity) {
        double accel = calculateAccel(measuredPosition, measuredVelocity);
        Log.i("accel", Double.toString(accel));

        return controller.update(measuredVelocity, accel);
    }

    public void reset() {
        controller.reset();
        accelTimeoutTimer = new ElapsedTime();
        accelSamples.clear();
    }
}