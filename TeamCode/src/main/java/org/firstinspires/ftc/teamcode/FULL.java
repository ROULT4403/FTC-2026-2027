package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FULL", group="Linear Opmode")
public class FULL extends LinearOpMode {

    Servo torreta;
    Limelight3A limelight;
    GoBildaPinpointDriver odo;
    DcMotor FWDright;
    DcMotor FWDleft;
    DcMotor BCKright;
    DcMotor BCKleft;
    DcMotor intake;
    double driveVelocityCap = 1;
    static final double VELOCIDAD_MANUAL = 0.005;  // controller speed per loop
    static final double GANANCIA = 0.002;          // Limelight correction strength
    static final double MIN_POS = 0.0;             // narrow these if the turret hits its limits
    static final double MAX_POS = 1.0;
    static final int TAG_OBJETIVO = 20;            // target tag ID, or -1 for any tag
    double posicionTorreta = 0.5;
    boolean autoAim = false;
    boolean yAnterior = false;

    @Override
    public void runOpMode() {

        //Agarra el pinpointer del control hub:
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        //Agarra el motor del intake desde el control hub:
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Agarra los motores del chasis desde el control hub:
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

        //Configuracion Odometria:
        odo.setOffsets(-155, 75, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        //Agarra el servo del control hub:
        torreta = hardwareMap.servo.get("Servo Torreta");

        //Agarra la limelight del control hub:
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //Inicia la limelight:
        limelight.pipelineSwitch(0);   // AprilTag pipeline
        limelight.start();
        torreta.setPosition(posicionTorreta);

        telemetry.addData("Estado", "Listo. Y = alternar auto-aim");
        telemetry.update();

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

            intake.setPower(1);

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

            //Denominador para que no pase la potencia del motor:
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

            // Prender apuntacion automatica en vez de manual:
            if (gamepad1.y && !yAnterior) autoAim = !autoAim;
            yAnterior = gamepad1.y;

            // control por control 0:
            if (gamepad1.right_bumper) posicionTorreta += VELOCIDAD_MANUAL;
            if (gamepad1.left_bumper)  posicionTorreta -= VELOCIDAD_MANUAL;
            posicionTorreta += gamepad1.right_stick_x * VELOCIDAD_MANUAL;

            // Limelight, sus cosas y apuntar,
            String estadoVision = "Apagado";
            if (autoAim) {
                estadoVision = "Sin objetivo";
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        if (tag.getFiducialId() == TAG_OBJETIVO) {
                            double tx = tag.getTargetXDegrees();
                            posicionTorreta -= tx * GANANCIA;
                            estadoVision = "Tag " + tag.getFiducialId() + " tx=" + String.format("%.1f", tx);
                            break;
                        }
                    }
                }
            }

            posicionTorreta = Range.clip(posicionTorreta, MIN_POS, MAX_POS);
            torreta.setPosition(posicionTorreta);

            //Telemetria de la Limelight:
            telemetry.addData("Modo", autoAim ? "AUTO (Limelight)" : "MANUAL");
            telemetry.addData("Limelight", estadoVision);
            telemetry.addData("Torreta", "%.3f", posicionTorreta);
            telemetry.update();

            //Telemetria del chasis:
            telemetry.addData("=== MONITOREO GO BILDA PINPOINT ===", "");
            telemetry.addData("Ángulo (Grados)", "%.2f°", Math.toDegrees(heading));
            telemetry.addData("Ángulo (Radianes)", "%.4f", heading);


            telemetry.addData("Pod Horizontal (X)", odo.getEncoderX());
            telemetry.addData("Pod Vertical (Y)", odo.getEncoderY());


            telemetry.addData("Posición X (mm)", "%.2f", odo.getPosX(DistanceUnit.MM));
            telemetry.addData("Posición Y (mm)", "%.2f", odo.getPosY(DistanceUnit.MM));

            telemetry.addData("---------------------------------", "");
            telemetry.addData("Botón Options", "Presiona para resetear el Norte");

            telemetry.update();


        }

        limelight.stop();

    }
}