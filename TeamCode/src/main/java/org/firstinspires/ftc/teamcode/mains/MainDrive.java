package org.firstinspires.ftc.teamcode.mains;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libraries.FieldCentric;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import static java.lang.Math.PI;

@TeleOp(name="Driving", group="drive")
public class MainDrive extends LinearOpMode implements TeleAuto {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor rl;
    private DcMotor rr;

    private DcMotor intake;
    private DcMotor shooter;
    private Servo pusher;
    private boolean shooterOn = false;
    private boolean shooterWait = false;
    private boolean pusherWait = false;

    private DcMotor wobbleMotor;
    private Servo wobbleServo;
    private double wobblePosition = -0.8;

    private boolean a1Wait = false;
    private boolean b1Wait = false;
    private boolean x1Wait = false;


    private BNO055IMU imu;

    private final FieldCentric centric = new FieldCentric();

    ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
    ScheduledFuture<?> pushEvent = null;
    ScheduledFuture<?> wobbleEvent = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        pusher = hardwareMap.get(Servo.class, "pusher");

        wobbleMotor = hardwareMap.get(DcMotor.class, "wobbleMotor");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");

        rl.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(params);

        final DcMotor[] motors = {fr, rr, rl, fl};
        final double[] motorAngles = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};

        centric.setUp(motors, motorAngles, imu);

        startScheduler();

        waitForStart();

        pusher.setPosition(1);

        while (opModeIsActive()) {
            if (gamepad1.back) {
                centric.resetAngle();
            }

            if (gamepad2.b && !shooterWait) {
                if (!shooterOn) {
                    shooter.setPower(0.8);
                    shooterOn = true;
                } else {
                    shooter.setPower(0);
                    shooterOn = false;
                }
                shooterWait = true;
            } else if (!gamepad2.b) {
                shooterWait = false;
            }

            telemetry.addData("Gamepad 2 B", gamepad2.b);
            telemetry.addData("Shooter Wait", shooterWait);
            telemetry.update();

            if (gamepad2.a && !pusherWait && pushEvent.isDone()) {
                pusher.setPosition(0.1);
                pusherWait = true;
                schedulePusher();
            } else {
                pusherWait = false;
            }

            wobbleMotor.setPower(gamepad2.left_stick_x);

            if ((gamepad2.right_stick_x > 0.1 || gamepad2.right_stick_x < -0.1) && wobbleEvent.isDone()) {
                wobblePosition += gamepad2.right_stick_x / 10;
                wobbleEvent = executorService.schedule(() -> {}, 100, TimeUnit.MILLISECONDS);
            }
            wobbleServo.setPosition(wobblePosition);

            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(intakePower);
        }
    }

    private void startScheduler() {
        pushEvent = executorService.schedule(() -> {}, 0, TimeUnit.MILLISECONDS);
        wobbleEvent = executorService.schedule(() -> {}, 0, TimeUnit.MILLISECONDS);
    }

    private void schedulePusher() {
        pushEvent = executorService.schedule(() -> pusher.setPosition(0.9), 1000, TimeUnit.MILLISECONDS);
    }
}