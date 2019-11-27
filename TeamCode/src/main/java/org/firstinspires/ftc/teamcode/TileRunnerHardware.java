package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TileRunnerHardware {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor rearLeft;
    public DcMotor rearRight;


    public void init(HardwareMap hardwareMap) {
        frontLeft   = hardwareMap.dcMotor.get("FL");
        frontRight  = hardwareMap.dcMotor.get("FR");
        rearLeft    = hardwareMap.dcMotor.get("BL");
        rearRight   = hardwareMap.dcMotor.get("BR");


        frontLeft.  setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.   setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.  setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight. setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.   setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.  setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
