package org.firstinspires.ftc.teamcode.mains;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.Alexi;
import org.firstinspires.ftc.teamcode.libraries.FieldCentric;
import org.firstinspires.ftc.teamcode.libraries.TeleAuto;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Driving", group="drive")
public class MainDrive extends LinearOpMode implements TeleAuto {
    private Alexi robot = new Alexi();

    private int[] wobblePositions = {0, -734, -1368, -1641};
    private double wobbleSpeed = 0;
    private int wobblePosition = 0;
    private boolean wobbleWait = false;

    private final FieldCentric centric = new FieldCentric();

    ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
    ScheduledFuture<?> pushEvent = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {
        robot.init(hardwareMap);

        boolean pusherWait = false;
        boolean shooterOn = false;

        double shooterSpeed = 0;
        double shooterTempSpeed = 4500;
        boolean shooterWait = false;

        boolean slow = false;
        boolean slowWait = false;
        double xSpeed = 0;
        double ySpeed = 0;
        double turnSpeed = 0;

        centric.setUp(robot.motors, robot.angles, robot.imu);

        startScheduler();

        waitForStart();

        robot.pusher.setPosition(0.9);

        while (opModeIsActive()) {
            if (gamepad1.back) {
                centric.resetAngle();
            }

            xSpeed = gamepad1.left_stick_x;
            ySpeed = -gamepad1.left_stick_y;
            turnSpeed = -gamepad1.right_stick_x;

            if (gamepad1.right_bumper && !slowWait) {
                slow = !slow;
                slowWait = true;
            } else if (!gamepad1.right_bumper) {
                slowWait = false;
            }

            if (slow) {
                xSpeed /= 2;
                ySpeed /= 2;
                turnSpeed /= 2;
            }

            centric.gyro(robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);
            centric.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

            if (gamepad2.y && !shooterWait) {
                if (!shooterOn) {
                    shooterSpeed = shooterTempSpeed;
                    shooterOn = true;
                } else {
                    shooterSpeed = 0;
                    shooterOn = false;
                }
                shooterWait = true;
            } else if (!gamepad2.y) {
                shooterWait = false;
            }

            telemetry.addData("Gamepad2 y", gamepad2.y);

            robot.shooter.setVelocity(shooterSpeed);

            if (gamepad1.dpad_up && !shooterWait) {
                shooterTempSpeed += 500;
                shooterWait = true;
            } else if (gamepad1.dpad_down && !shooterWait) {
                shooterTempSpeed -= 500;
                shooterWait = true;
            } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                shooterWait = false;
            }

            telemetry.addData("Shooter temp speed", shooterTempSpeed);
            telemetry.addData("Shooter speed", robot.shooter.getVelocity());

            if (robot.shooter.getVelocity() >= shooterTempSpeed) {
                telemetry.addLine("READY FIRE");
            } else {
                telemetry.addLine("No fire");
            }

            if (gamepad2.a && !pusherWait && pushEvent.isDone()) {
                robot.pusher.setPosition(0.1);
                pusherWait = true;
                schedulePusher();
            } else {
                pusherWait = false;
            }

            robot.wobbleMotor.setPower(gamepad2.left_stick_x);

            if (gamepad2.x) {
                robot.wobbleServo.setPosition(-1);
            } else if (gamepad2.b) {
                robot.wobbleServo.setPosition(1);
            }

            if (gamepad2.dpad_right && wobblePosition < 3 && !wobbleWait) {
                wobblePosition++;
                wobbleWait = true;
            } else if (gamepad2.dpad_left && wobblePosition > 0 && !wobbleWait) {
                wobblePosition--;
                wobbleWait = true;
            } else if (!gamepad2.dpad_right && !gamepad2.dpad_left) { wobbleWait = false; }

            robot.wobbleMotor.setTargetPosition(wobblePositions[wobblePosition]);
            if (robot.wobbleMotor.isBusy()) {
                robot.wobbleMotor.setVelocity(800);
            } else {
                robot.wobbleMotor.setVelocity(0);
            }

            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            robot.intake.setPower(intakePower);

            telemetry.update();
        }
    }

    private void startScheduler() {
        pushEvent = executorService.schedule(() -> {}, 0, TimeUnit.MILLISECONDS);
    }

    private void schedulePusher() {
        pushEvent = executorService.schedule(() -> robot.pusher.setPosition(0.9), 1000, TimeUnit.MILLISECONDS);
    }
}