package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class VeloPIDTuner extends LinearOpMode {
    public static double MOTOR_TICKS_PER_REV = 383.6;
    public static double MOTOR_MAX_RPM = 435;
    public static double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    public static double TESTING_MAX_SPEED = 0.9 * MOTOR_MAX_RPM;
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

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(30, 0, 2, 14);

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

    private double currentTargetVelo = 0.0;
    private double lastTargetVelo = 0.0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        myMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

        double SPEED_RANGE = TESTING_MAX_SPEED - TESTING_MIN_SPEED;

        ElapsedTime externalTimer = new ElapsedTime();

        StateMachine stateMachine = new StateMachineBuilder<State>()
                .state(State.RAMPING_UP)
                .transitionTimed(STATE1_RAMPING_UP_DURATION)
                .onEnter(externalTimer::reset)
                .loop(() -> {
                    double progress = externalTimer.seconds() / STATE1_RAMPING_UP_DURATION;
                    double target = progress * SPEED_RANGE + TESTING_MIN_SPEED;

                    currentTargetVelo = rpmToTicksPerSecond(target);
                })

                .state(State.COASTING_1)
                .transitionTimed(STATE2_COASTING_1_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(TESTING_MAX_SPEED))

                .state(State.RAMPING_DOWN)
                .transitionTimed(STATE3_RAMPING_DOWN_DURATION)
                .onEnter(externalTimer::reset)
                .loop(() -> {
                    double progress = externalTimer.seconds() / STATE3_RAMPING_DOWN_DURATION;
                    double target = TESTING_MAX_SPEED - progress * SPEED_RANGE;

                    currentTargetVelo = rpmToTicksPerSecond(target);
                })

                .state(State.COASTING_2)
                .transitionTimed(STATE4_COASTING_2_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(TESTING_MIN_SPEED))

                .state(State.RANDOM_1)
                .transitionTimed(STATE5_RANDOM_1_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(Math.random() * SPEED_RANGE + TESTING_MIN_SPEED))

                .state(State.RANDOM_2)
                .transitionTimed(STATE6_RANDOM_2_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(Math.random() * SPEED_RANGE + TESTING_MIN_SPEED))

                .state(State.RANDOM_3)
                .transitionTimed(STATE7_RANDOM_3_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(Math.random() * SPEED_RANGE + TESTING_MIN_SPEED))

                .state(State.REST)
                .transitionTimed(STATE8_REST_DURATION)
                .onEnter(() -> currentTargetVelo = 0)

                .exit(State.RAMPING_UP)

                .build();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        stateMachine.setLooping(true);
        stateMachine.start();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("currentState", stateMachine.getState());

            if(currentTargetVelo != lastTargetVelo) myMotor.setVelocity(currentTargetVelo);
            lastTargetVelo = currentTargetVelo;

            telemetry.addData("targetVelocity", currentTargetVelo);

            double motorVelo = myMotor.getVelocity();
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", currentTargetVelo - motorVelo);

            telemetry.addData("upperBound", rpmToTicksPerSecond(TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);

            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(myMotor, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            stateMachine.update();
            telemetry.update();
        }
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
