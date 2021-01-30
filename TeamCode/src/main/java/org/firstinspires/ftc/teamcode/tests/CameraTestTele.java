package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="Camera Test TeleOp", group="CamTest")
@Disabled
public class CameraTestTele extends LinearOpMode {
    private T265Camera camera = null;

    public void runOpMode() {
        camera = new T265Camera(new Transform2d(), 0.8, hardwareMap.appContext);

        waitForStart();

        if (opModeIsActive()) {
            camera.start();
            sleep(5000);
            camera.getLastReceivedCameraUpdate();
            camera.stop();
            sleep(2000);
        }
    }
}
