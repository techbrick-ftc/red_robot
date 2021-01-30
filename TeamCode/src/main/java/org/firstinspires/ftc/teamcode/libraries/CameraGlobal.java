package org.firstinspires.ftc.teamcode.libraries;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

public class CameraGlobal {
    private static T265Camera camera = null;

    public static T265Camera getCamera(HardwareMap hardwareMap) {
        if (camera == null) {
            camera = new T265Camera(new Transform2d(), 0.8, hardwareMap.appContext);
        }
        try {
            Thread.sleep(500);
        } catch (Exception ignored) {}
        if (!camera.isStarted()) { camera.start(); }
        return camera;
    }
}
