package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Finder", group="test")
public class MotorFinder extends LinearOpMode {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor rl;
    private DcMotor rr;

    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        waitForStart();

        if (opModeIsActive()) {
            runMotor(fl);
            runMotor(fr);
            runMotor(rl);
            runMotor(rr);
        }
    }

    private void runMotor(DcMotor motor) {
        motor.setPower(1);
        sleep(2000);
        motor.setPower(0);
    }
}
