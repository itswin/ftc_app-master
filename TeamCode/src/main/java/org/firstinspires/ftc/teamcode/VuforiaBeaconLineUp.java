/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.PIController;
import org.firstinspires.ftc.robotcontroller.internal.RobotMovement;
import org.firstinspires.ftc.robotcontroller.internal.TurnMovement;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="Vuforia Beacon Line Up", group ="Vuforia")
public class VuforiaBeaconLineUp extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;

    int[] legosToolsOrientation = new int[]{90, -90};
    int[] gearsWheelsOrientation = new int[]{0, -0};

    private PIController piController;
    private boolean needToTurn;
    private double turnError;
    private RobotMovement robotMovement;
    private TurnMovement turnMovement;
    private boolean lineUpX;
    private int linedUpX;
    private boolean lineUpY;
    private int linedUpY;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() throws InterruptedException
    {
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");

        // Reverse Right if forwards, reverse left if backwards
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
////        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // .0032   .000017
        piController = new PIController(.0016, 0.00013, 0.00023, 0.000012);
        robotMovement = new RobotMovement(0,0);

        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATjJBiD/////AAAAGc0JoedLjk5flVb2gExO3UVJCpOq5U4cyH9czcMyX5C8h+1AWXo7A0CU24r/IVeoC+7Te9zwJkX6IjHv5c77UNqrsyerM7pbjywj6/2NlzSUwb3jtEd9APhY5cOoSibb5NDRFM9beUWt0k4HuFMaw5OIZRs5YWge7KaJt5SzhqEFMQ6Loo8eugB9BBbPfuV3d7u4sQZBAKeRsR9mmnfvFJTUHHgcPlALU/rJBgw40AeFFvChjzNhwtlWYymeM/0173jH7JB2dyhoNtn/9byIUQzMw8KtaXbD3IfFJySLgJWmYjaA7cKdboL0nvkOoZNFMm2yqenbUDe/CEIMkhAsKjS3sgX4t6Fq+8gkhSnOS/Vd";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables visionTargets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheelsTarget = visionTargets.get(0);
        wheelsTarget.setName("Wheels");  // Wheels

        VuforiaTrackable toolsTarget  = visionTargets.get(1);
        toolsTarget.setName("Tools");  // Tools

        VuforiaTrackable legosTarget = visionTargets.get(2);
        legosTarget.setName("Legos");  // Legos

        VuforiaTrackable gearsTarget  = visionTargets.get(3);
        gearsTarget.setName("Gears");  // Gears

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(visionTargets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        float mmVisionTargetZOffset = 5.75f * mmPerInch;
        float mmPhoneZOffset = 5.5f * mmPerInch;

        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, -mmFTCFieldWidth/12, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gearsTarget.setLocation(gearsTargetLocationOnField);
        RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));

        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, mmFTCFieldWidth/4, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        toolsTarget.setLocation(toolsTargetLocationOnField);
        RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

       /*
        * To place the Wheels and Legos Targets on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(mmFTCFieldWidth/12, mmFTCFieldWidth/2, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheelsTarget.setLocation(wheelsTargetLocationOnField);
        RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsTargetLocationOnField));

        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(-mmFTCFieldWidth/4, mmFTCFieldWidth/2, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legosTarget.setLocation(legosTargetLocationOnField);
        RobotLog.ii(TAG, "Legos Target=%s", format(legosTargetLocationOnField));

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,mmPhoneZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 180, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)wheelsTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legosTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gearsTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)toolsTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        visionTargets.activate();

        needToTurn = false;
        turnError = 0;
        lineUpX = false;
        linedUpX = 0;
        lineUpY = false;
        linedUpY = 0;
        turnMovement = new TurnMovement(false, 0);
        robotMovement = new RobotMovement(0, 0);
        while (opModeIsActive()) {
            String visibleTarget = "";

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                // Add beacon to telemetry if visible
                if(((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    visibleTarget = trackable.getName();
                    telemetry.addData(visibleTarget, "Visible");
                }

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", myFormat(lastLocation));

                if(!visibleTarget.equals("")) {
                    robotMovement = piController.processLocation(lastLocation, visibleTarget);
                    turnMovement = piController.processOrientation(lastLocation, visibleTarget);
                    telemetry.addData("Move", robotMovement);
                    telemetry.addData("Turn", turnMovement);
                }
            } else {
                telemetry.addData("Pos", "Unknown");
            }

            if(turnMovement.getNeedToTurn()) {
                piController.reset();
                turn(turnMovement.getTurnError());
            } else if(robotMovement.getMmXDistance() != 0) {
                double[] motorPowers = piController.processXMovement(robotMovement.getMmXDistance());
                for (int i = 0; i < 5; i++) {
                    frontLeftMotor.setPower(motorPowers[0]);
                    backLeftMotor.setPower(motorPowers[1]);
                    frontRightMotor.setPower(motorPowers[2]);
                    backRightMotor.setPower(motorPowers[3]);
                }
            } else if(robotMovement.getMmYDistance() != 0 && !lineUpY) {
                double[] motorPowers = piController.processYMovement(robotMovement.getMmYDistance());
                for (int i = 0; i < 5; i++) {
                    frontLeftMotor.setPower(motorPowers[0]);
                    backLeftMotor.setPower(motorPowers[1]);
                    frontRightMotor.setPower(motorPowers[2]);
                    backRightMotor.setPower(motorPowers[3]);
                }
            } else {
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }

            if(!visibleTarget.equals("") && robotMovement.getMmXDistance() == 0 && !lineUpX) {
                linedUpX++;
                if(linedUpX >= 25) {
                    lineUpX = true;
                    piController.reset();
                }
            }

            if(!visibleTarget.equals("") && robotMovement.getMmYDistance() == 0 && !lineUpY) {
                linedUpY++;
                if(linedUpY >= 25) {
                    lineUpY = true;
                    piController.reset();
                }
            }

            telemetry.update();
            idle();
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    private String myFormat(OpenGLMatrix transformationMatrix)
    {
        VectorF translation = transformationMatrix.getTranslation();

        Orientation orientation = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        return String.format("%n  %s%n  %s", orientation.toString(), translation.toString());
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
        for(int i = 0; i < 10 * Math.abs(degreesToTurn); i++) {
            frontRightMotor.setPower(degreesToTurn > 0 ? .15 : -.15);
            backRightMotor.setPower(degreesToTurn > 0 ? .15 : -.15);
            frontLeftMotor.setPower(degreesToTurn > 0 ? -.15 : .15);
            backLeftMotor.setPower(degreesToTurn > 0 ? -.15 : .15);
        }

        needToTurn = false;
    }
}