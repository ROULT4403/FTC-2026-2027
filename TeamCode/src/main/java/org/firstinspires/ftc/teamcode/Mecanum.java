package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Mecanum", group="Linear Opmode")
public class Mecanum extends LinearOpMode {
    DcMotor FWDright;
    GoBildaPinpointDriver odo;
    DcMotor FWDleft;
    DcMotor BCKright;
    DcMotor BCKleft;
    double driveVelocityCap = 1;

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

        odo.setOffsets(-155, 75, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();


            if (gamepad1.a) {
                odo.resetPosAndIMU(); // Resetea el IMU a cero
            }


            if (gamepad1.left_bumper) {
                driveVelocityCap = .4;

            } else {
                driveVelocityCap = 1;
            }


            // Agarra el ángulo del pinpoint:
            double headingRadians = odo.getHeading(AngleUnit.RADIANS);
            double headingDegrees = Math.toDegrees(headingRadians);


            // Leemos el estado de la conexión y datos del dispositivo
            String status = odo.getDeviceStatus().toString();


            //Agarra los joysticks para el movimiento:
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;


            //Agarra el angulo del pinpoint:
            double heading = odo.getHeading(AngleUnit.RADIANS);


            // Field Centric:
            double roty = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
            double rotx = strafe * Math.cos(-heading) - forward * Math.sin(-heading);


            //Denominador para que sobrepase la potencia del motor:
            double denominator = Math.max(Math.abs(roty) + Math.abs(rotx) + Math.abs(rotation), 1.0);


            //Formula mecanum:
            double FWDleftPWR  = ((roty + rotx + rotation) / denominator) * driveVelocityCap;
            double FWDrightPWR = ((roty - rotx - rotation) / denominator) * driveVelocityCap;
            double BCKleftPWR  = ((roty - rotx + rotation) / denominator) * driveVelocityCap;
            double BCKrightPWR = ((roty + rotx - rotation) / denominator) * driveVelocityCap;


            //Poder a los motores:
            FWDleft.setPower(FWDleftPWR);
            FWDright.setPower(FWDrightPWR);
            BCKleft.setPower(BCKleftPWR);
            BCKright.setPower(BCKrightPWR);










            telemetry.addData("=== MONITOREO GO BILDA PINPOINT ===", "");
            telemetry.addData("Ángulo (Grados)", "%.2f°", Math.toDegrees(heading));
            telemetry.addData("Ángulo (Radianes)", "%.4f", heading);

            // Imprime los tics crudos que están contando tus ruedas odométricas
            telemetry.addData("Pod Horizontal (X)", odo.getEncoderX());
            telemetry.addData("Pod Vertical (Y)", odo.getEncoderY());

            // Si quieres ver las coordenadas calculadas en el campo (en milímetros)
            telemetry.addData("Posición X (mm)", "%.2f", odo.getPosX(DistanceUnit.MM));
            telemetry.addData("Posición Y (mm)", "%.2f", odo.getPosY(DistanceUnit.MM));

            telemetry.addData("---------------------------------", "");
            telemetry.addData("Botón Options", "Presiona para resetear el Norte");

            telemetry.update();


        }


 }
}
