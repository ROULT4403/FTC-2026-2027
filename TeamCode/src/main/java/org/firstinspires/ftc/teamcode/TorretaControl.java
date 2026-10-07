package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// deja ver como funcionan los commits 2 :v
@TeleOp(name = "Torreta Control", group = "TeleOp")
public class TorretaControl extends LinearOpMode {

    Servo torreta;
    Limelight3A limelight;

    // velocidades y eso
    static final double VELOCIDAD_MANUAL = 0.005;  // controller speed per loop
    static final double GANANCIA = 0.002;          // Limelight correction strength
    static final double MIN_POS = 0.0;             // narrow these if the turret hits its limits
    static final double MAX_POS = 1.0;
    static final int TAG_OBJETIVO = 20;            // target tag ID, or -1 for any tag

    double posicionTorreta = 0.5;
    boolean autoAim = false;
    boolean yAnterior = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        torreta = hardwareMap.servo.get("Servo Torreta");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);   // AprilTag pipeline
        limelight.start();
        torreta.setPosition(posicionTorreta);

        telemetry.addData("Estado", "Listo. Y = alternar auto-aim");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // preder apuntacion automatica envez de manual
            if (gamepad1.y && !yAnterior) autoAim = !autoAim;
            yAnterior = gamepad1.y;

            // control por control 0:
            if (gamepad1.right_bumper) posicionTorreta += VELOCIDAD_MANUAL;
            if (gamepad1.left_bumper)  posicionTorreta -= VELOCIDAD_MANUAL;
            posicionTorreta += gamepad1.right_stick_x * VELOCIDAD_MANUAL;

            // Limelight y sus cosas y apuntar,
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

            telemetry.addData("Modo", autoAim ? "AUTO (Limelight)" : "MANUAL");
            telemetry.addData("Limelight", estadoVision);
            telemetry.addData("Torreta", "%.3f", posicionTorreta);
            telemetry.update();
        }

        limelight.stop();
    }
}

// mejor codigo que el codigo de windows 10