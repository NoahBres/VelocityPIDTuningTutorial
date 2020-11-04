package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@TeleOp
public class VeloPIDTuner extends LinearOpMode {
    public static double MOTOR_TICKS_PER_REV = 383.6;
    public static double MOTOR_MAX_RPM = 435;
    public static double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    public static double TESTING_MAX_SPEED = 0.95 * MOTOR_MAX_RPM;
    public static double TESTING_MIN_SPEED = 0.3 * MOTOR_MAX_RPM;

    // These are prefixed with "STATE1", "STATE2", etc. because Dashboard displays variables in
    // alphabetical order. Thus, we preserve the actual order of the process
    public static double STATE1_RAMPING_UP_DURATION = 3.5;
    public static double STATE2_COASTING_1_DURATION = 4;
    public static double STATE3_RAMPING_DOWN_DURATION = 2;
    public static double STATE4_COASTING_2_DURATION = 2;
    public static double STATE5_RANDOM_1_DURATION = 2;
    public static double STATE6_RANDOM_2_DURATION = 2;
    public static double STATE7_RANDOM_3_DURATION = 2;
    public static double STATE8_REST_DURATION = 1;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF());

    enum State {
        RAMPING_UP,
        COASTING_1,
        RAMPING_DOWN,
        COASTING_2,
        RANDOM_1,
        RANDOM_2,
        RANDOM_3,
        REST
    }

    private double lastKp = 0.0;
    private double lastKi = 0.0;
    private double lastKd = 0.0;
    private double lastKf = getMotorVelocityF();

    private State currentState;
    private State lastState;
    private ElapsedTime timer;

    private double currentTargetVelo = 0.0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.RAMPING_UP;
        lastState = null;
        timer = new ElapsedTime();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("currentState", currentState);

            switch (currentState) {
                case RAMPING_UP:
                    double progress1 = timer.seconds() / STATE1_RAMPING_UP_DURATION;
                    double target1 = progress1 * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;

                    currentTargetVelo = rpmToTicksPerSecond(target1);
                    setVelocity(myMotor, currentTargetVelo);

                    if (progress1 >= 1) {
                        currentState = State.COASTING_1;
                        timer.reset();
                    }
                    break;
                case COASTING_1:
                    if (lastState != State.COASTING_1) {
                        currentTargetVelo = rpmToTicksPerSecond(TESTING_MAX_SPEED);
                        setVelocity(myMotor, currentTargetVelo);

                        lastState = State.COASTING_1;
                    }

                    if (timer.seconds() >= STATE2_COASTING_1_DURATION) {
                        currentState = State.RAMPING_DOWN;
                        timer.reset();
                    }
                    break;
                case RAMPING_DOWN:
                    double progress2 = timer.seconds() / STATE3_RAMPING_DOWN_DURATION;
                    double target2 = TESTING_MAX_SPEED - progress2 * (TESTING_MAX_SPEED - TESTING_MIN_SPEED);

                    currentTargetVelo = rpmToTicksPerSecond(target2);
                    setVelocity(myMotor, currentTargetVelo);

                    if (progress2 >= 1) {
                        currentState = State.COASTING_2;
                        timer.reset();
                    }
                    break;
                case COASTING_2:
                    if (lastState != State.COASTING_2) {
                        setVelocity(myMotor, rpmToTicksPerSecond(TESTING_MIN_SPEED));

                        lastState = State.COASTING_2;
                    }

                    currentTargetVelo = rpmToTicksPerSecond(TESTING_MIN_SPEED);

                    if (timer.seconds() >= STATE4_COASTING_2_DURATION) {
//                        currentState = State.RAMPING_UP;
                        currentState = State.RANDOM_1;
                        timer.reset();
                    }
                    break;
                case RANDOM_1:
                    if (lastState != State.RANDOM_1) {
                        currentTargetVelo = rpmToTicksPerSecond(
                                Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED
                        );
                        setVelocity(myMotor, currentTargetVelo);

                        lastState = State.RANDOM_1;
                    }

                    if (timer.seconds() >= STATE5_RANDOM_1_DURATION) {
                        currentState = State.RANDOM_2;
                        timer.reset();
                    }
                    break;
                case RANDOM_2:
                    if (lastState != State.RANDOM_2) {
                        currentTargetVelo = rpmToTicksPerSecond(
                                Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED
                        );
                        setVelocity(myMotor, currentTargetVelo);

                        lastState = State.RANDOM_2;
                    }

                    if (timer.seconds() >= STATE6_RANDOM_2_DURATION) {
                        currentState = State.RANDOM_3;
                        timer.reset();
                    }
                    break;
                case RANDOM_3:
                    if (lastState != State.RANDOM_3) {
                        currentTargetVelo = rpmToTicksPerSecond(
                                Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED
                        );
                        setVelocity(myMotor, currentTargetVelo);

                        lastState = State.RANDOM_3;
                    }

                    if (timer.seconds() >= STATE7_RANDOM_3_DURATION) {
                        currentState = State.REST;
                        timer.reset();
                    }
                    break;
                case REST:
                    if (lastState != State.REST) {
                        currentTargetVelo = 0;
                        setVelocity(myMotor, currentTargetVelo);

                        lastState = State.REST;
                    }

                    if (timer.seconds() >= STATE8_REST_DURATION) {
                        currentState = State.RAMPING_UP;
                        timer.reset();
                    }
                    break;
            }

            printVelocites(myMotor, currentTargetVelo);

            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            telemetry.update();
        }
    }

    private void printVelocites(DcMotorEx motor, double target) {
        telemetry.addData("targetVelocity", target);

        double motorVelo = motor.getVelocity();
        telemetry.addData("velocity", motorVelo);
        telemetry.addData("error", target - motorVelo);
    }

    private void setVelocity(DcMotorEx motor, double power) {
        Log.i("power", Double.toString(power));
        motor.setVelocity(power);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV);
    }
}
