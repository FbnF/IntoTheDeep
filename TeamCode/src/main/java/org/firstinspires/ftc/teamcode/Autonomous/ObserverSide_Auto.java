package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class ObserverSide_Auto extends LinearOpMode {

    private ElapsedTime AutoTimer = new ElapsedTime();
    private ArmControl armControl;
    private Gripper gripper;
    private SliderControl sliderControl;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // - - - Setting up Arm motors - - - //
        armControl = new ArmControl(this);
        armControl.init(hardwareMap);
        // Set the Hang servo up to put the blocking plate in place to hold the arm up
        armControl.setHangServoUp();
        //armControl.setDesArmPosDeg(-5);

        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl(this);
        sliderControl.init(hardwareMap);

        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper(this);
        gripper.init(hardwareMap);
        //Gripper closed state
        gripper.setGripperClosed();
		//Gripper holder parallel to the bar
        gripper.setGripperHolderParallel();
        gripper.setAnglerInit();


        // Define starting position
        Pose2d startPos = new Pose2d(8, 53, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Pose2d SamplePickUpPos1 = new Pose2d(35.5, 22.2, Math.toRadians(0));
        Pose2d SamplePickUpPos2 = new Pose2d(35.6, 15.2, Math.toRadians(0));

        Pose2d SpecimenDropoffPos = new Pose2d(32, 65, Math.toRadians(0));
        Pose2d SpecimenDropoffPos2 = new Pose2d(32.7, 68, Math.toRadians(0));
        Pose2d SpecimenDropoffPos3 = new Pose2d(32.7, 62, Math.toRadians(0));

        Pose2d SampleDropoffPos = new Pose2d(28, 26, -Math.toRadians(135));
        Pose2d SpecimenPickupPos = new Pose2d(22, 44, -Math.toRadians(135));
        Pose2d ParkPos = new Pose2d(10, 11, Math.toRadians(0));

        // Define the trajectory sequence for the Observer side
        TrajectorySequence ObserverTrajectory = drive.trajectorySequenceBuilder(startPos)

                //--------------- Specimen 1 Operation ------------------ //
                // Step 1: Set the gripper and arm in the right position for Specimen drop off
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(72);})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerSample();})
				.UNSTABLE_addTemporalMarkerOffset(0.1,()->{gripper.setGripperHolderPerpendicular();})

                // Step 2: Move the robot to the Specimen drop off position and move forward,
                // then set the Arm down to prepare for placing the Specimen
                .lineToLinearHeading(SpecimenDropoffPos)
                .waitSeconds(0.1)
                .forward(5.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {armControl.setDesArmPosDeg(45);})
                .waitSeconds(0.1)

                // Step 3: Move backward and open the Gripper to place and release the Specimen.
                // At the same time, drop the arm all the way down and set its power to zero
                // afterwards
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> armControl.setArmPower(0))
				.UNSTABLE_addTemporalMarkerOffset(0.3, () -> gripper.setAnglerSpecimen())
                .waitSeconds(0.4)

                //--------------- Sample 1 pick up and drop off Operation ------------------ //

                // Step 4: Move to Sample 1 to pick up the Sample 1
                .lineToLinearHeading(SamplePickUpPos1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setAnglerSample())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setGripperClosed())
                .waitSeconds(0.4)

                //Step 5: Goto SampleDropoffPos to drop off Sample 1
                .lineToLinearHeading(SampleDropoffPos)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setAnglerSpecimen())
                .waitSeconds(0.28)

                //--------------- Sample 2 pick up and drop off Operation ------------------ //

                // Step 6: Pick up Sample 2
                .lineToLinearHeading(SamplePickUpPos2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setAnglerSample())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> gripper.setGripperClosed())
                .waitSeconds(0.5)

                // Step 7: Goto Sample drop off position  to drop off Sample 2
                .lineToLinearHeading(SampleDropoffPos)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setAnglerSample())
                .waitSeconds(0.28)

                //--------------- Specimen 2 Operation ------------------ //
                // Step 8: Pick up Specimen 2
                // Picking up Specimen 2 from SpecimenPickupPos
                .lineToLinearHeading(SpecimenPickupPos)
                //.waitSeconds(3.0)
                //.forward(2)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> gripper.setGripperClosed())
                .waitSeconds(1)
                //.UNSTABLE_addTemporalMarkerOffset(0.35, () -> gripper.setAnglerSpecimen())

                //.back(5)
                // Step 7: Raise the Arm to prepare for Specimen 2 drop off attempt
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {armControl.setDesArmPosDeg(72);})
                .waitSeconds(0.3)

                // Step 8: Specimen 2 drop off attempt
                .lineToLinearHeading(SpecimenDropoffPos2)
                .forward(5.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {armControl.setDesArmPosDeg(45);})
                .waitSeconds(0.1)
                // Step 9: Move backward and open the Gripper to place and release the Specimen.
                // At the same time, drop the arm all the way down and set its power to zero
                // afterwards
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))
                //.UNSTABLE_addTemporalMarkerOffset(0.4, () -> armControl.setArmPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setAnglerSample())
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> sliderControl.setDesSliderLen(0))
                .waitSeconds(0.4)

                //--------------- Specimen 3 Operation ------------------ //
                // Picking up Specimen 3 from SpecimenPickupPos
                .lineToLinearHeading(SpecimenPickupPos)
                //.waitSeconds(3.0)
                //.forward(2)

                .UNSTABLE_addTemporalMarkerOffset(1, () -> gripper.setGripperClosed())
                .waitSeconds(1)

                //.UNSTABLE_addTemporalMarkerOffset(0.35, () -> gripper.setAnglerPosition(0.25))
                //.back(5)
                // Step 7: Raise the Arm to prepare for Specimen  3 drop off attempt
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {armControl.setDesArmPosDeg(72);})
                //.UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setAnglerSample())
                //.turn(Math.toRadians(135))
                .waitSeconds(0.3)

                // Step 8: Specimen 3 drop off attempt
                .lineToLinearHeading(SpecimenDropoffPos3)
                .forward(5.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {armControl.setDesArmPosDeg(45);})
                .waitSeconds(0.1)
                // Step 9: Move backward and open the Gripper to place and release the Specimen.
                // At the same time, drop the arm all the way down and set its power to zero
                // afterwards
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> sliderControl.setDesSliderLen(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> armControl.setArmPower(0))
                .waitSeconds(0.4)

                // Step 12: Go to ParkPos for final parking
                .lineToLinearHeading(ParkPos)
                .waitSeconds(2)

                // Final build for this trajectory
                .build();

        // Wait for start signal
        waitForStart();

        // Execute the trajectory sequence
        drive.followTrajectorySequence(ObserverTrajectory);



    }
}