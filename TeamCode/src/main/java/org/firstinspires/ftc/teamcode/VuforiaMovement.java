package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

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

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.VuforiaLocation.TAG;

/**
 * Created by Winston on 10/8/2016.
 */
@Autonomous(name="Vuforia Movement", group="Vuforia")
public class VuforiaMovement extends LinearOpMode {

    private VuforiaLocalizerImplSubclass vuforia;
    private ColorBlobDetector colorDetector;
    private OpenGLMatrix lastLocation = null;

    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor rollerMotor;

    private boolean hitRed;
    private String directionToHit;
    private boolean isButtonHit;
    private int directionFoundInARow;

    private boolean needToTurn;
    private double degreesToTurn;

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

        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        colorDetector = new ColorBlobDetector();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATjJBiD/////AAAAGc0JoedLjk5flVb2gExO3UVJCpOq5U4cyH9czcMyX5C8h+1AWXo7A0CU24r/IVeoC+7Te9zwJkX6IjHv5c77UNqrsyerM7pbjywj6/2NlzSUwb3jtEd9APhY5cOoSibb5NDRFM9beUWt0k4HuFMaw5OIZRs5YWge7KaJt5SzhqEFMQ6Loo8eugB9BBbPfuV3d7u4sQZBAKeRsR9mmnfvFJTUHHgcPlALU/rJBgw40AeFFvChjzNhwtlWYymeM/0173jH7JB2dyhoNtn/9byIUQzMw8KtaXbD3IfFJySLgJWmYjaA7cKdboL0nvkOoZNFMm2yqenbUDe/CEIMkhAsKjS3sgX4t6Fq+8gkhSnOS/Vd";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new VuforiaLocalizerImplSubclass(parameters);

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
        telemetry.addData("OpenCV", Core.NATIVE_LIBRARY_NAME);
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        beacons.activate();

        hitRed = true;
        isButtonHit = false;
        directionFoundInARow = 0;
        directionToHit = "";

        movingToFirstBeacon = false;
        liningUpWithFirstBeacon = false;
        movingToSecondBeacon = false;
        liningUpWithSecondBeacon = false;
        while(opModeIsActive()) {
            String visibleTarget = "";
            Mat img = null;
            Mat croppedImg = null;
            Point beaconImageCenter = null;

            if(movingToFirstBeacon) {
                // TODO Estimate distance to the beacon from a point TBD
                // TODO Estimate distance to move forward and turn to face the beacon until second movement set
                // Move this outside the loop?

                // Move forward until you see the beacon
                while(movingToFirstBeacon) {
                    // Move in increments to minimize the times you check the trackables
                    for(int i = 0; i < 50; i++) {
                        frontRightMotor.setPower(1);
                        backRightMotor.setPower(1);
                        frontLeftMotor.setPower(1);
                        backLeftMotor.setPower(1);
                    }

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

                    // Move to the beacon until the beacon is in sight
                    if(lastLocation != null) {
                        movingToFirstBeacon = false; // Only execute this once
                    }
                }
            }

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

                processMovement(movement);
            }

            if(movingToSecondBeacon) {
                // TODO Estimate the movements/distance from the first beacon to the second
                movingToSecondBeacon = false; // Only execute this once
            }


            if(vuforia.rgb != null && !isButtonHit) {
                Bitmap bmp = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
                bmp.copyPixelsFromBuffer(vuforia.rgb.getPixels());

                img = new Mat();
                Utils.bitmapToMat(bmp, img);
            }

            for(VuforiaTrackable beacon : beacons) {
                // Add beacon to telemetry if visible
                if(((VuforiaTrackableDefaultListener)beacon.getListener()).isVisible()) {
                    visibleTarget = beacon.getName();
                    telemetry.addData(visibleTarget, "Visible");
                }

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)beacon.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)beacon.getListener()).getRawPose();

                if(pose != null) {
                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);

                    // Corners of beacon image in camera image
                    Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,92,0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,92,0));
                    Vec2F lowerLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,-92,0));
                    Vec2F lowerRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,-92,0));

                    VectorF translation = pose.getTranslation();
                    /** First argument is get(1) if phone is vertical
                     First argument is get(0) if phone is horizontal */
                    // DOES NOT WORK???
                    degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData("Degrees-", degreesToTurn);
                    // TODO Check degreee turning threshold
                    if(Math.abs(degreesToTurn) > 15) {
                        // Turn after doing calculating transformations
                        needToTurn = true;
                    }

                    if(img != null && !isButtonHit) {
                        telemetry.addData(beacon.getName() + "-Translation", translation);
                        telemetry.addData(beacon.getName() + "-Degrees", degreesToTurn);

                        // Vectors are stored (y,x).  Coordinate system starts in top right
                        int height = (int)(lowerLeft.getData()[0] - upperLeft.getData()[0]);
                        int width = (int)(upperLeft.getData()[1] - upperRight.getData()[1]);

                        int rowStart = (int)upperRight.getData()[0] - height < 0 ? 0 : (int)upperRight.getData()[0] - height;
                        int rowEnd = rowStart + height > img.rows() ? img.rows() - 1 : rowStart + height;
                        int colStart = (int)upperRight.getData()[1] < 0 ? 0 : (int)upperRight.getData()[1];
                        int colEnd = colStart + width > img.cols() ? img.cols() - 1 : colStart + width;

                        telemetry.addData("Target Location", "");
                        telemetry.addData("[" + upperLeft.getData()[0] + "," + upperLeft.getData()[1] + "]", "[" + upperRight.getData()[0] + "," + upperRight.getData()[1] + "]");
                        telemetry.addData("[" + lowerLeft.getData()[0] + "," + lowerLeft.getData()[1] + "]", "[" + lowerRight.getData()[0] + "," + lowerRight.getData()[1] + "]");

                        telemetry.addData(colStart + "", rowStart);
                        telemetry.addData(colEnd + "", rowEnd);

                        // Crop the image to look only at the beacon
                        // TODO Verify beacon is in cropped image
                        // NEED TO CHECK BEACON HEIGHT FOR INCLUSION IN CROPPED IMAGE
                        croppedImg = img.submat(rowStart, rowEnd, colStart, colEnd);
                    }
                }
            }

            // Process the rgb image
            if(croppedImg != null && !isButtonHit) {
                // Find the color of the beacon you need to hit
                if(hitRed) {
                    colorDetector.setHsvColor(new Scalar(180,240,240)); // Red detector, needs verification with beacon
                } else {
                    colorDetector.setHsvColor(new Scalar(25,255,185)); // Blue detector, needs verification with beacon
                }
                colorDetector.process(croppedImg);

                // Calculate the center of the blob detected
                Point beaconToHitCenter = null;
                List<Moments> blueMu = new ArrayList<>(colorDetector.getContours().size());
                for (int i = 0; i < colorDetector.getContours().size(); i++) {
                    blueMu.add(Imgproc.moments(colorDetector.getContours().get(i), false));
                    Moments p = blueMu.get(i);
                    int x = (int) (p.get_m10() / p.get_m00());
                    int y = (int) (p.get_m01() / p.get_m00());
                    beaconToHitCenter = new Point(x, y);
                }

                // Find the color of the beacon you are not hitting
                if(hitRed) {
                    colorDetector.setHsvColor(new Scalar(25,255,185)); // Blue detector, needs verification with beacon
                } else {
                    colorDetector.setHsvColor(new Scalar(180,240,240)); // Red detector, needs verification with beacon
                }
                colorDetector.process(croppedImg);

                // Calculate the center of the blob detected
                Point secondReferenceCenter = null;
                List<Moments> redMu = new ArrayList<>(colorDetector.getContours().size());
                for (int i = 0; i < colorDetector.getContours().size(); i++) {
                    redMu.add(Imgproc.moments(colorDetector.getContours().get(i), false));
                    Moments p = redMu.get(i);
                    int x = (int) (p.get_m10() / p.get_m00());
                    int y = (int) (p.get_m01() / p.get_m00());
                    secondReferenceCenter = new Point(x, y);
                }

                // Use the two centers of the blobs to determine which direction to hit
                if (beaconToHitCenter != null && secondReferenceCenter != null && !isButtonHit && !needToTurn) {
                    // (!isButtonHit) Only hit the button once
                    // (!needToTurn) Do not hit the button if the robot is not straight centered
//                    hitBeaconButton(isLeft(center, beaconImageCenter));
                    if(isLeft(beaconToHitCenter, secondReferenceCenter)) {
                        if(!directionToHit.equals("Left")) {
                            directionFoundInARow = 0;
                        }
                        directionFoundInARow++;
                        directionToHit = "Left";
                    } else {
                        if(!directionToHit.equals("Right")) {
                            directionFoundInARow = 0;
                        }
                        directionFoundInARow++;
                        directionToHit = "Right";
                    }
                }

                // Find the color five times in a row before hitting it
                if(directionFoundInARow >= 3) {
                    isButtonHit = true;
                }
            }

            if(isButtonHit) {
                telemetry.addData("Hit Button-", directionToHit);
            }

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
                    telemetry.addData("Move", processLocation(lastLocation, visibleTarget));
                }
            } else {
                telemetry.addData("Pos", "Unknown");
            }

            telemetry.update();
            idle();
        }
    }

    /**
     * Returns if point 1 is to the left of point 2
     * Assume x coordinates increase from left to right
     * @param p1 a point
     * @param p2 a point
     * @return whether point 1 is to the left of point 2
     */
    private boolean isLeft(Point p1, Point p2)
    {
        return p1.x < p2.x;
    }

    /**
     * Hit the beacon button that is on the left
     * @param isLeft should the bot hit the left button
     */
    public void hitBeaconButton(boolean isLeft)
    {
        isButtonHit = true;

        // TODO Tweak turn degrees
        turn(isLeft ? -15 : 15);

        // TODO Tweak loop length and motor power
        // Move forward
        for(int i = 0; i < 100; i++) {
            frontRightMotor.setPower(1);
            backRightMotor.setPower(1);
            frontLeftMotor.setPower(1);
            backLeftMotor.setPower(1);
        }

        // TODO Tweak turn degrees (again)
        turn(isLeft ? 15 : -15);
    }

    /**
     * Turn the bot by the given degrees
     * Positive is CW, negative is CCW
     * Best guess at loop length and motor power
     * @param degreesToTurn number of degrees to turn
     */
    private void turn(double degreesToTurn)
    {
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

    /**
     * Uses the last location of the robot and the visible beacon to center the robot with the beacon
     * @param lastLocation Last location of the robot
     * @param visibleTarget Name of the target that is seen
     * @return List of movements to make to center the robot with the beacon target
     */
    private RobotMovement processLocation(OpenGLMatrix lastLocation, String visibleTarget)
    {
        float mmXDistance = 0;
        float mmYDistance = 0;

        if(lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            if(visibleTarget.equals("Wheels")) {
                if(translation.getData()[1] <= 1180) {
                    mmYDistance = 1180 - translation.getData()[1];
                } else if(translation.getData()[1] >= 1220) {
                    mmYDistance = 1220 - translation.getData()[1];
                }

                if(translation.getData()[0] >= 340) {
                    mmXDistance = 340 - translation.getData()[0];
                } else if(translation.getData()[0] <= 300) {
                    mmXDistance = 300 - translation.getData()[0];
                }
            } else if(visibleTarget.equals("Tools")) {
                if(translation.getData()[0] >= -1180) {
                    mmYDistance = 1180 + translation.getData()[0];
                } else if(translation.getData()[0] <= -1220) {
                    mmYDistance = 1220 + translation.getData()[0];
                }

                if(translation.getData()[1] >= 940) {
                    mmXDistance = 940 - translation.getData()[1];
                } else if(translation.getData()[1] <= 900) {
                    mmXDistance = 900 - translation.getData()[1];
                }
            } else if(visibleTarget.equals("Legos")) {
                if(translation.getData()[1] <= 1180) {
                    mmYDistance = 1180 - translation.getData()[1];
                } else if(translation.getData()[1] >= 1220) {
                    mmYDistance = 1220 - translation.getData()[1];
                }

                if(translation.getData()[0] >= -855) {
                    mmXDistance = 855 + translation.getData()[0];
                } else if(translation.getData()[0] <= -895) {
                    mmXDistance = 895 + translation.getData()[0];
                }
            } else if(visibleTarget.equals("Gears")) {
                if(translation.getData()[0] >= -1180) {
                    mmYDistance = 1180 + translation.getData()[0];
                } else if(translation.getData()[0] <= -1220) {
                    mmYDistance = 1220 + translation.getData()[0];
                }

                if(translation.getData()[1] >= -260) {
                    mmXDistance = 260 + translation.getData()[1];
                } else if(translation.getData()[1] <= -300) {
                    mmXDistance = 300 + translation.getData()[1];
                }
            }
        }

        return new RobotMovement(mmXDistance, mmYDistance);
    }

    private void processMovement(RobotMovement movement)
    {
        // TODO process Y and X AXES movements
    }
}