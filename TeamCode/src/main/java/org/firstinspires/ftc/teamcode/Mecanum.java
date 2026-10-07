package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Mecanum extends LinearOpMode {
    DcMotor FWDright;
    GoBildaPinpointDriver odo;
    DcMotor FWDleft;
    DcMotor BCKright;
    DcMotor BCKleft;
    public void drive(double forward, double strafe){


    }

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        FWDleft = hardwareMap.get(DcMotor.class, "FWDleft");
        FWDright = hardwareMap.get(DcMotor.class, "FWDright");
        BCKleft = hardwareMap.get(DcMotor.class, "BCKleft");
        BCKright = hardwareMap.get(DcMotor.class, "BCKright");

        FWDleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FWDleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FWDleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FWDright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FWDright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FWDright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BCKright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BCKright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BCKright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BCKleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BCKleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BCKleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        IMU odo = hardwareMap.get(IMU.class, "imu");

        odo.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        double heading = odo.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        double FWDleftPWR = forward + strafe + rotation;
        double FWDrightPWR = forward - strafe + rotation;
        double BCKleftPWR = forward - strafe - rotation;
        double BCKrightPWR = forward + strafe - rotation;

        double maxPWR = 1.0;
        double maxSPEED = 1.0;

        maxPWR = Math.max(maxPWR, Math.abs(FWDleftPWR));
        maxPWR = Math.max(maxPWR, Math.abs(FWDrightPWR));
        maxPWR = Math.max(maxPWR, Math.abs(BCKleftPWR));
        maxPWR = Math.max(maxPWR, Math.abs(BCKrightPWR));

        FWDleft.setPower(maxSPEED * (FWDleftPWR / maxPWR));
        FWDright.setPower(maxSPEED * (FWDrightPWR / maxPWR));
        BCKleft.setPower(maxSPEED * (BCKleftPWR / maxPWR));
        BCKright.setPower(maxSPEED * (BCKrightPWR / maxPWR));

        waitForStart();
        while (opModeIsActive()) {





        }


 }
}
