package org.firstinspires.ftc.samuelv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class DeviceMap {
    public CameraName camera_one;
    public CameraName camera_two;

    public DeviceMap(HardwareMap map) {
        camera_one = map.get(WebcamName.class, "camera1");
        camera_two = map.get(WebcamName.class, "camera2");
    }
}
