package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class EasyOpenCVImportable {
    private OpenCvCamera webCamera;
    private OpenCvInternalCamera phoneCamera;

    private CameraType cameraType;

    public EasyOpenCVImportable(CameraType cameraType, OpenCvCamera webCamera) throws Exception {
        if (!cameraType.equals(CameraType.WEBCAM)) { throw new Exception("The camera type and camera passed do not match! Please check your code."); }
    }

    public EasyOpenCVImportable(CameraType cameraType, OpenCvInternalCamera phoneCamera) throws Exception {
        if (!cameraType.equals(CameraType.PHONE)) { throw new Exception("The camera type and camera passed do not match! Please check your code."); }
    }

    enum CameraType {
        PHONE,
        WEBCAM
    }
}
