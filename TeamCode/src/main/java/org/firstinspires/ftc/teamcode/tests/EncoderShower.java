package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Encoder Shower", group="test")
@Disabled
public class EncoderShower extends OpMode {
    final String motorName = "wobbleMotor";
    DcMotor motor;

    public void init() {
        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop() {
        telemetry.addData("Encoder", motor.getCurrentPosition());
    }
}
