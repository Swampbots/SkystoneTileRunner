package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Tile Runner", group = "TeleOp")
public class TileRunnerTeleOp extends OpMode{

    TileRunnerHardware hardware = new TileRunnerHardware();

    double flPower = 0.0;
    double frPower = 0.0;
    double rlPower = 0.0;
    double rrPower = 0.0;

    public void init() {
        hardware.init(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();
    }

    public void loop(){

        flPower = gamepad1.left_stick_y;
        rrPower = gamepad1.left_stick_y;

        rlPower = gamepad1.right_stick_y;
        frPower = gamepad1.right_stick_y;

        hardware.frontRight.setPower(frPower);
        hardware.rearLeft.setPower(rlPower);
        hardware.frontLeft.setPower(frPower);
        hardware.rearRight.setPower(rrPower);

    }
}
