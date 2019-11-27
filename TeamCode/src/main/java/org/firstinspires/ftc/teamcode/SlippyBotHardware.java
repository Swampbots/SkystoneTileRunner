package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlippyBotHardware {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor rearLeft;
    public DcMotor rearRight;


    public void init(HardwareMap hardwareMap) {
        frontLeft   = hardwareMap.dcMotor.get("fl_drive");
        frontRight  = hardwareMap.dcMotor.get("fr_drive");
        rearLeft    = hardwareMap.dcMotor.get("rl_drive");
        rearRight   = hardwareMap.dcMotor.get("rr_drive");


        frontLeft.  setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.   setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.  setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight. setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.   setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.  setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
