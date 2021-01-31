package org.firstinspires.ftc.teamcode.mains;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libraries.Alexi;
import org.firstinspires.ftc.teamcode.libraries.AutonomousFunctions;
import org.firstinspires.ftc.teamcode.libraries.CameraAuto;
import org.firstinspires.ftc.teamcode.libraries.CameraType;
import org.firstinspires.ftc.teamcode.libraries.EasyOpenCVImportable;
import org.firstinspires.ftc.teamcode.libraries.TeleAuto;

@Autonomous(name="Blue Left Drop", group="Blue")
public class BlueLeftDrop extends LinearOpMode implements TeleAuto {
    Alexi robot = new Alexi();
    CameraAuto drive = new CameraAuto();
    AutonomousFunctions auto = new AutonomousFunctions(robot, this);
    EasyOpenCVImportable openCV = new EasyOpenCVImportable();
    ElapsedTime et = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.wobbleServo.setPosition(-1);

        drive.setUp(robot.motors, robot.angles, robot.camera, robot.imu, AxesReference.EXTRINSIC, telemetry);
        drive.setPose(new Pose2d(-56, 17, new Rotation2d()));

        openCV.init(CameraType.WEBCAM, hardwareMap);
        openCV.setBox(80, 110, 70, 50);
        openCV.setThresholds(135, 148);

        waitForStart();

        if (opModeIsActive()) {
            robot.camera.start();
            openCV.startDetection();
            while (openCV.getDetection().equals(EasyOpenCVImportable.RingNumber.UNKNOWN)) { idle(); }
            et.reset();
            while (et.seconds() < 1) { idle(); }
            telemetry.addData("Detected", openCV.getDetection());
            telemetry.update();
            if (openCV.getDetection().equals(EasyOpenCVImportable.RingNumber.NONE)) {
                openCV.stopDetection();
                drive.goTo(10, 45, 0, this);
                auto.armDownAndDrop();
                drive.goToPosition(10, 0, this);
            }
        }
    }
}
