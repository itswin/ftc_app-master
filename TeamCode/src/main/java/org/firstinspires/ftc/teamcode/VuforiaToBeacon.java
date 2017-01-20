package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcontroller.internal.PIController;
import org.firstinspires.ftc.robotcontroller.internal.RobotMovement;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import static org.firstinspires.ftc.teamcode.VuforiaLocation.TAG;

/**
 * Created by Winston on 10/8/2016.
 */
@Autonomous(name="Vuforia To Beacon", group="Vuforia")
public class VuforiaToBeacon extends LinearOpMode {

    private VuforiaLocalizerImplSubclass vuforia;
    private OpenGLMatrix lastLocation = null;

    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor rollerMotor;

    private boolean needToTurn;
    private double degreesToTurn;

    private PIController piController;
    private boolean movingToFirstBeacon;
    private boolean liningUpWithFirstBeacon;
    private boolean movingToSecondBeacon;
    private boolean liningUpWithSecondBeacon;

    public void runOpMode() throws InterruptedException
    {
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        rollerMotor = hardwareMap.dcMotor.get("rollerMotor");

        // Reverse Right if forwards, reverse left if backwards
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATjJBiD/////AAAAGc0JoedLjk5flVb2gExO3UVJCpOq5U4cyH9czcMyX5C8h+1AWXo7A0CU24r/IVeoC+7Te9zwJkX6IjHv5c77UNqrsyerM7pbjywj6/2NlzSUwb3jtEd9APhY5cOoSibb5NDRFM9beUWt0k4HuFMaw5OIZRs5YWge7KaJt5SzhqEFMQ6Loo8eugB9BBbPfuV3d7u4sQZBAKeRsR9mmnfvFJTUHHgcPlALU/rJBgw40AeFFvChjzNhwtlWYymeM/0173jH7JB2dyhoNtn/9byIUQzMw8KtaXbD3IfFJySLgJWmYjaA7cKdboL0nvkOoZNFMm2yqenbUDe/CEIMkhAsKjS3sgX4t6Fq+8gkhSnOS/Vd";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new VuforiaLocalizerImplSubclass(parameters);

        piController = new PIController(.0016, 0.00013, 0.00023, 0.000012);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        float mmVisionTargetZOffset = 5.75f * mmPerInch;

        // Initialize the location of the targets and phone on the field
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(mmFTCFieldWidth/12, mmFTCFieldWidth/2, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        beacons.get(0).setLocation(wheelsTargetLocationOnField);
        RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsTargetLocationOnField));

        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, mmFTCFieldWidth/4, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        beacons.get(1).setLocation(toolsTargetLocationOnField);
        RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(-mmFTCFieldWidth/4, mmFTCFieldWidth/2, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        beacons.get(2).setLocation(legosTargetLocationOnField);
        RobotLog.ii(TAG, "Legos Target=%s", format(legosTargetLocationOnField));

        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, -mmFTCFieldWidth/12, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        beacons.get(3).setLocation(gearsTargetLocationOnField);
        RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "Phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)beacons.get(0).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)beacons.get(1).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)beacons.get(2).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)beacons.get(3).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        beacons.activate();

        movingToFirstBeacon = false;
        liningUpWithFirstBeacon = true;
        movingToSecondBeacon = false;
        liningUpWithSecondBeacon = false;
        while(opModeIsActive()) {
            String visibleTarget = "";

            if(movingToFirstBeacon) {
                // TODO Estimate distance to the beacon from a point TBD
                // TODO Estimate distance to move forward and turn to face the beacon until second movement set
                for(int i = 0; i < 100; i++) {
                    frontLeftMotor.setPower(.25);
                    frontRightMotor.setPower(.25);
                    backLeftMotor.setPower(.25);
                    backRightMotor.setPower(.25);
                }
                movingToFirstBeacon = false;

            }
            /*
            while(liningUpWithFirstBeacon) {
                for(VuforiaTrackable beacon : beacons) {
                    // Add beacon to telemetry if visible
                    if (((VuforiaTrackableDefaultListener) beacon.getListener()).isVisible()) {
                        visibleTarget = beacon.getName();
                        telemetry.addData(visibleTarget, "Visible");
                    }

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) beacon.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                }

                RobotMovement movement = processLocation(lastLocation, visibleTarget);
                if(movement.isNoMovement()) {
                    liningUpWithFirstBeacon = false;
                }

                if(!visibleTarget.equals("")) {
                    telemetry.addData("Move", processLocation(lastLocation, visibleTarget));
                }

                processMovement(movement);
            } */

//            if(movingToSecondBeacon) {
//                // TODO Estimate the movements/distance from the first beacon to the second
//                movingToSecondBeacon = false; // Only execute this once
//            }
//
//            while(liningUpWithSecondBeacon) {
//                for(VuforiaTrackable beacon : beacons) {
//                    // Add beacon to telemetry if visible
//                    if (((VuforiaTrackableDefaultListener) beacon.getListener()).isVisible()) {
//                        visibleTarget = beacon.getName();
//                        telemetry.addData(visibleTarget, "Visible");
//                    }
//
//                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) beacon.getListener()).getUpdatedRobotLocation();
//                    if (robotLocationTransform != null) {
//                        lastLocation = robotLocationTransform;
//                    }
//                }
//
//                RobotMovement movement = processLocation(lastLocation, visibleTarget);
//                if(movement.isNoMovement()) {
//                    liningUpWithSecondBeacon = false;
//                }
//
//                processMovement(movement);
//            }


//            if(needToTurn) {
//                turn(degreesToTurn);
//                telemetry.addData("Turn-", degreesToTurn);
//            }

            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", myFormat(lastLocation));

                if(!visibleTarget.equals("")) {
                    telemetry.addData("Move", piController.processLocation(lastLocation, visibleTarget));
                }
            } else {
                telemetry.addData("Pos", "Unknown");
            }

            telemetry.update();
            idle();
        }
    }

    /**
     * Turn the bot by the given degrees
     * Positive is CW, negative is CCW
     * Best guess at loop length and motor power
     * @param degreesToTurn number of degrees to turn
     */
    private void turn(double degreesToTurn)
    {
        // 580 may be too much, 520 may be too little
        for(int i = 0; i < 580 * Math.abs(degreesToTurn); i++) {
            frontRightMotor.setPower(degreesToTurn > 0 ? 1 : -1);
            backRightMotor.setPower(degreesToTurn > 0 ? 1 : -1);
            frontLeftMotor.setPower(degreesToTurn > 0 ? -1 : 1);
            backLeftMotor.setPower(degreesToTurn > 0 ? -1 : 1);
        }

        needToTurn = false;
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    private String format(OpenGLMatrix transformationMatrix) { return transformationMatrix.formatAsTransform(); }

    private String myFormat(OpenGLMatrix transformationMatrix)
    {
        VectorF translation = transformationMatrix.getTranslation();

        Orientation orientation = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        return String.format("%n  %s%n  %s", orientation.toString(), translation.toString());
    }
}