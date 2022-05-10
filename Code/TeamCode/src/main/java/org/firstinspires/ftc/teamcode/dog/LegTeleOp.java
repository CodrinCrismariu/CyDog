package org.firstinspires.ftc.teamcode.dog;

import static org.firstinspires.ftc.teamcode.RobotConstants.defaultDriveSpeed;
import static org.firstinspires.ftc.teamcode.dog.Constants.speed;
import static org.firstinspires.ftc.teamcode.dog.Constants.x1;
import static org.firstinspires.ftc.teamcode.dog.Constants.x2;
import static org.firstinspires.ftc.teamcode.dog.Constants.x3;
import static org.firstinspires.ftc.teamcode.dog.Constants.y1;
import static org.firstinspires.ftc.teamcode.dog.Constants.y2;
import static org.firstinspires.ftc.teamcode.dog.Constants.y3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;

import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.List;

// Coefficienti pentru a alinia servo-urile pe pozitia corecta

@Config
class FrontRight {
    public static double coeff1 = 240;
    public static double coeff2 = 300;
    public static double coeff3 = 50;
    public static double coeff4 = -300;
    public static double coeff5 = 0.5;
    public static double coeff6 = 1;
}

@Config
class FrontLeft {
    public static double coeff1 = 235;
    public static double coeff2 = -300;
    public static double coeff3 = 60;
    public static double coeff4 = 300;
    public static double coeff5 = 0.5;
    public static double coeff6 = 1;
}

@Config
class BackRight {
    public static double coeff1 = 250;
    public static double coeff2 = 300;
    public static double coeff3 = 55;
    public static double coeff4 = -300;
    public static double coeff5 = 0.47;
    public static double coeff6 = 1;
}

@Config
class BackLeft {
    public static double coeff1 = 250;
    public static double coeff2 = -300;
    public static double coeff3 = 65;
    public static double coeff4 = 300;
    public static double coeff5 = 0.5;
    public static double coeff6 = 1;
}

@Config
class Constants {
    public static double speed = 0.17, x1 = 100, x2 = 145, x3 = 145, y1 = -20, y2 = -20, y3 = 50, z = 40;
}

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LegTeleOp")
public class LegTeleOp extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    Leg frontRight, frontLeft, backRight, backLeft;
    OpenCvCamera camera = null;
    
    // deschide camera si transmite datele catre laptop prin FTCDashBoard
    void openCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.FRONT, cameraMonitorViewId);
        camera.setPipeline(new PipeLine());

        // ------------------ OpenCv code
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // ------------------ Tzeapa frate
            }

        });

        // transmit camera image to laptop
        FtcDashboard.getInstance().startCameraStream(camera, 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // 0 -> rotation servo
        // 1 -> height servo
        // 2 -> tilt servo
        
        
        // Initializam picioarele 
        frontRight = new Leg(hardwareMap, "fr0", "fr1", "fr2",
                FrontRight.coeff1, FrontRight.coeff2,
                FrontRight.coeff3, FrontRight.coeff4, FrontRight.coeff5, FrontRight.coeff6);
        frontLeft = new Leg(hardwareMap, "fl0", "fl1", "fl2",
                FrontLeft.coeff1, FrontLeft.coeff2,
                FrontLeft.coeff3, FrontLeft.coeff4, FrontLeft.coeff5, FrontLeft.coeff6);
        backRight = new Leg(hardwareMap, "br0", "br1", "br2",
                BackRight.coeff1, BackRight.coeff2,
                BackRight.coeff3, BackRight.coeff4, BackRight.coeff5, BackRight.coeff6);
        backLeft = new Leg(hardwareMap, "bl0", "bl1", "bl2",
                BackLeft.coeff1, BackLeft.coeff2,
                BackLeft.coeff3, BackLeft.coeff4, BackLeft.coeff5, BackLeft.coeff6);

        backLeft.goTo(148, 30);
        backRight.goTo(148, 30);
        frontLeft.goTo(148, 30);
        frontRight.goTo(148, 30);

        openCamera();

        // Robot class

        waitForStart();

        Thread l1 = null, l2 = null, l3 = null, l4 = null;

        // ------------------ Main Thread
        while (opModeIsActive()) {
            
            
            // cand robotul merge in fata
            if (gamepad1.left_stick_y < -0.2) {
                Thread t1 = new Thread(() -> {
                    backRight.interpolateTo(x3, y3, 0, 2000 * speed);
                    backRight.interpolateTo(x1, y1, 0, 1000 * speed);
                    backRight.interpolateTo(x2, y2, 0, 1000 * speed);
                    backRight.interpolateTo(x3, y3, 0, 4000 * speed);
                });

                Thread t2 = new Thread(() -> {
                    backLeft.interpolateTo(x3, y3, 0, 6000 * speed);
                    backLeft.interpolateTo(x1, y1, 0, 1000 * speed);
                    backLeft.interpolateTo(x2, y2, 0, 1000 * speed);
                });

                Thread t3 = new Thread(() -> {
                    frontRight.interpolateTo(x3, y3, 0, 4000 * speed);
                    frontRight.interpolateTo(x1, y1, 0, 1000 * speed);
                    frontRight.interpolateTo(x2, y2, 0, 1000 * speed);
                    frontRight.interpolateTo(x3, y3, 0, 2000 * speed);
                });

                t1.start();
                t2.start();
                t3.start();

                frontLeft.interpolateTo(x1, y1, 0, 1000 * speed);
                frontLeft.interpolateTo(x2, y2, 0, 1000 * speed);
                frontLeft.interpolateTo(x3, y3, 0, 6000 * speed);

                t1.join();
                t2.join();
                t3.join();
            } else if (gamepad1.left_stick_y > 0.2) {
                // cand robotul merge in spate
                Thread t1 = new Thread(() -> {
                    backRight.interpolateTo(x3, y2, 0, 2000 * speed);
                    backRight.interpolateTo(x1, y3, 0, 1000 * speed);
                    backRight.interpolateTo(x2, y3, 0, 1000 * speed);
                    backRight.interpolateTo(x3, y2, 0, 4000 * speed);
                });

                Thread t2 = new Thread(() -> {
                    backLeft.interpolateTo(x3, y2, 0, 6000 * speed);
                    backLeft.interpolateTo(x1, y3, 0, 1000 * speed);
                    backLeft.interpolateTo(x2, y3, 0, 1000 * speed);
                });

                Thread t3 = new Thread(() -> {
                    frontRight.interpolateTo(x3, y2, 0, 4000 * speed);
                    frontRight.interpolateTo(x1, y3, 0, 1000 * speed);
                    frontRight.interpolateTo(x2, y3, 0, 1000 * speed);
                    frontRight.interpolateTo(x3, y2, 0, 2000 * speed);
                });

                t1.start();
                t2.start();
                t3.start();

                frontLeft.interpolateTo(x1, y3, 1000 * speed);
                frontLeft.interpolateTo(x2, y3, 1000 * speed);
                frontLeft.interpolateTo(x3, y2, 6000 * speed);

                t1.join();
                t2.join();
                t3.join();
            } else if(gamepad1.left_stick_x > 0.2) {
                // cand robotul se invarte
                Thread t1 = new Thread(() -> {
                    backRight.interpolateTo(x3, 20, 0, 2000 * speed);
                       backRight.interpolateTo(x1, 20, Constants.z, 1000 * speed);
                    backRight.interpolateTo(x2, 20, Constants.z, 1000 * speed);
                    backRight.interpolateTo(x3, 20, 0, 4000 * speed);
                });

                Thread t2 = new Thread(() -> {
                    backLeft.interpolateTo(x3, 20, 0,6000 * speed);
                    backLeft.interpolateTo(x1, 20, Constants.z, 1000 * speed);
                    backLeft.interpolateTo(x2, 20, Constants.z, 1000 * speed);
                });

                Thread t3 = new Thread(() -> {
                    frontRight.interpolateTo(x3, 20, 0, 4000 * speed);
                    frontRight.interpolateTo(x1, 20, Constants.z, 1000 * speed);
                    frontRight.interpolateTo(x2, 20, Constants.z, 1000 * speed);
                    frontRight.interpolateTo(x3, 20, 0,2000 * speed);
                });

                t1.start();
                t2.start();
                t3.start();

                frontLeft.interpolateTo(x1, 20, Constants.z, 1000 * speed);
                frontLeft.interpolateTo(x2, 20, Constants.z,1000 * speed);
                frontLeft.interpolateTo(x3, 20, 0,6000 * speed);

                t1.join();
                t2.join();
                t3.join();
            } else if(gamepad1.left_stick_x < -0.2) {
                // cand robotul se invarte
                Thread t1 = new Thread(() -> {
                    backRight.interpolateTo(x3, 20, 0, 2000 * speed);
                    backRight.interpolateTo(x1, 20, -Constants.z, 1000 * speed);
                    backRight.interpolateTo(x2, 20, -Constants.z, 1000 * speed);
                    backRight.interpolateTo(x3, 20, 0, 4000 * speed);
                });

                Thread t2 = new Thread(() -> {
                    backLeft.interpolateTo(x3, 20, 0,6000 * speed);
                    backLeft.interpolateTo(x1, 20, -Constants.z, 1000 * speed);
                    backLeft.interpolateTo(x2, 20, -Constants.z, 1000 * speed);
                });

                Thread t3 = new Thread(() -> {
                    frontRight.interpolateTo(x3, 20, 0, 4000 * speed);
                    frontRight.interpolateTo(x1, 20, -Constants.z, 1000 * speed);
                    frontRight.interpolateTo(x2, 20, -Constants.z, 1000 * speed);
                    frontRight.interpolateTo(x3, 20, 0,2000 * speed);
                });

                t1.start();
                t2.start();
                t3.start();

                frontLeft.interpolateTo(x1, 20, -Constants.z, 1000 * speed);
                frontLeft.interpolateTo(x2, 20, -Constants.z,1000 * speed);
                frontLeft.interpolateTo(x3, 20, 0,6000 * speed);

                t1.join();
                t2.join();
                t3.join();
            } else {
                frontLeft.goTo(148, 30, 0);
                frontRight.goTo(148, 30, 0);
                backRight.goTo(148, 30, 0);
                backLeft.goTo(148, 30, 0);
                telemetry.update();
//
            }
        }

    }

    class PipeLine extends OpenCvPipeline {
        boolean viewportPaused = false;

        @Override
        public Mat processFrame(Mat input) {
            // we can process the frame for image recognition
            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                camera.pauseViewport();
            } else {
                camera.resumeViewport();
            }
        }
    }
}
