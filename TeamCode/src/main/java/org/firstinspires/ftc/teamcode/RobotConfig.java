package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class RobotConfig {
    public static double motorSpeed = 0.2;
    public static double gamepad1_leftstick_x = 0;
    public static double gamepad1_leftstick_y = 0;
    public static double gamepad1_rightstick_x = 0;
    public static DcMotor.ZeroPowerBehavior braking = DcMotor.ZeroPowerBehavior.FLOAT;
}
