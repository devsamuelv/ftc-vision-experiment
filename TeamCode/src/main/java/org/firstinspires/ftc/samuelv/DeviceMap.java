package org.firstinspires.ftc.samuelv;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class DeviceMap {
    public CameraName camera_one;
    public CameraName camera_two;
    public DcMotorEx front_left_drive;
    public DcMotorEx front_right_drive;
    public DcMotorEx back_left_drive;
    public DcMotorEx back_right_drive;

    public DeviceMap(HardwareMap map) {
        camera_one = map.get(WebcamName.class, "Webcam 1");
        camera_two = map.get(WebcamName.class, "Webcam 2");

        front_left_drive = map.get(DcMotorEx.class, "leftF_drive");
        front_right_drive = map.get(DcMotorEx.class, "rightF_drive");
        back_left_drive = map.get(DcMotorEx.class, "leftB_drive");
        back_right_drive = map.get(DcMotorEx.class, "rightB_drive");
    }
}
