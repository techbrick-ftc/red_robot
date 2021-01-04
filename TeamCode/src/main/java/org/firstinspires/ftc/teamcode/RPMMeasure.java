package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="RPM Measure", group="test")
public class RPMMeasure extends LinearOpMode {
    // CHANGE THESE
    private final String motorName = "testMotor";   // Name of motor in config
    private final double motorSpeed = 1;          // Power of motor to run at
    private final long revTime = 500;               // Time in millis to allow the motor to speed up
    private final int ticksPerRotation = 2786;      // Ticks per rotation of output shaft
    private final int sampleSize = 10;            // How many times for the code to get rotation speed

    // NOT THESE
    private DcMotor driveMotor;
    private final List<Double> rotSpeeds = new ArrayList<>();

    public void runOpMode() {
        int tickOffset;
        int tickDelta;
        double spr;
        double rps;
        double rpm;

        driveMotor = hardwareMap.get(DcMotor.class, motorName);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        driveMotor.setPower(motorSpeed);
        sleep(revTime);
        tickOffset = driveMotor.getCurrentPosition();

        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested()) {
            tickDelta = driveMotor.getCurrentPosition() - tickOffset;
            if (tickDelta >= ticksPerRotation) {
                if (rotSpeeds.size() >= sampleSize) {
                    rotSpeeds.clear();
                }

                rotSpeeds.add(et.seconds());
                spr = rotSpeeds.stream().mapToDouble(c -> c).sum() / rotSpeeds.size();
                rps = 1 / spr;
                rpm = rps * 60;

                telemetry.addData("RPM", rpm);
                telemetry.update();
                et.reset();
                tickOffset = driveMotor.getCurrentPosition();
            }
        }
    }
}
