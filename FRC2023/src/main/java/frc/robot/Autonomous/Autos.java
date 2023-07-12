// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import frc.robot.Constants;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public final class Autos {

        public static final PathPlannerTrajectory PickupToFlingBlue = PathPlanner.loadPath("PickupToFlingBlue",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static final PathPlannerTrajectory ScoreAndFront = PathPlanner.loadPath("Score and Front",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static final PathPlannerTrajectory PickupToFlingRed = PathPlanner.loadPath("PickupToFlingRed",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static final PathPlannerTrajectory PickupToBalanceNoFlingBlue = PathPlanner.loadPath(
                        "PickupToBalanceNoFlingBlue",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static final PathPlannerTrajectory PickupToBalanceNoFlingRed = PathPlanner.loadPath(
                        "PickupToBalanceNoFlingRed",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static final PathPlannerTrajectory PickupToFlingBottomBlue = PathPlanner.loadPath(
                        "PickupToFlingBottomBlue",
                        Constants.Drivebase.PATH_CONSTRAINTS);

        public static final List<PathPlannerTrajectory> BlueBottomMidPickup = PathPlanner.loadPathGroup(
                        "Blue Bottom Mid Pickup",
                        new PathConstraints(4.0, 2.0));

        public static final List<PathPlannerTrajectory> RedBottomMidPickup = PathPlanner.loadPathGroup(
                        "Red Bottom Mid Pickup",
                        new PathConstraints(4.0, 2.0));
        public static final PathPlannerTrajectory PickupToFlingBottomRed = PathPlanner.loadPath(
                        "PickupToFlingBottomRed",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static final PathPlannerTrajectory ScoreAndFrontBlue = PathPlanner.loadPath(
                        "Score and Front Blue",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static final PathPlannerTrajectory BlueTopToCharging = PathPlanner.loadPath(
                        "Blue Top to Charging",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static final PathPlannerTrajectory RedTopToCharging = PathPlanner.loadPath(
                        "Red Top to Charging",
                        Constants.Drivebase.PATH_CONSTRAINTS);

        public static List<PathPlannerTrajectory> BlueDriveOutBottom = PathPlanner.loadPathGroup(
                        "Blue Drive Out From Bottom",
                        Constants.Drivebase.PATH_CONSTRAINTS);

        public static List<PathPlannerTrajectory> RedDriveOutBottom = PathPlanner.loadPathGroup(
                        "Red Drive Out From Bottom",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static List<PathPlannerTrajectory> BlueTop2Cycle = PathPlanner.loadPathGroup(
                        "Blue Top Two Cycle Slowdown",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static List<PathPlannerTrajectory> Charging = PathPlanner.loadPathGroup("Charging One Cycle",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static List<PathPlannerTrajectory> BlueBottom2Cycle = PathPlanner.loadPathGroup("Blue Bottom Two Cycle",
                        Constants.Drivebase.PATH_CONSTRAINTS, new PathConstraints(1.0, 1.0),
                        Constants.Drivebase.PATH_CONSTRAINTS, Constants.Drivebase.PATH_CONSTRAINTS,
                        new PathConstraints(1.0, 1.0), Constants.Drivebase.PATH_CONSTRAINTS);
        public static List<PathPlannerTrajectory> BlueBottom2CycleTag = PathPlanner.loadPathGroup(
                        "Blue Bottom Two Cycle Tag",
                        new PathConstraints(2.0, 2.0), new PathConstraints(4.0, 3.0));
        public static List<PathPlannerTrajectory> RedBottom2CycleTag = PathPlanner.loadPathGroup(
                        "Red Bottom Two Cycle Tag",
                        new PathConstraints(2.0, 2.0), new PathConstraints(4.0, 3.0));

        public static List<PathPlannerTrajectory> RedTop2Cycle = PathPlanner.loadPathGroup("Red Top Two Cycle Slowdown",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static List<PathPlannerTrajectory> RedBottom2Cycle = PathPlanner.loadPathGroup("Blue Bottom Two Cycle",
                        Constants.Drivebase.PATH_CONSTRAINTS, new PathConstraints(1.0, 1.0),
                        Constants.Drivebase.PATH_CONSTRAINTS, Constants.Drivebase.PATH_CONSTRAINTS,
                        new PathConstraints(1.0, 1.0), Constants.Drivebase.PATH_CONSTRAINTS);

        public static PathPlannerTrajectory ThreeMetersForwardBlue = PathPlanner.loadPath("Three Meters Forward Blue",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory ThreeMetersForwardRed = PathPlanner.loadPath("Three Meters Forward Blue",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory ThreeMetersBack = PathPlanner.loadPath("Three Meters Back",
                        Constants.Drivebase.PATH_CONSTRAINTS);

        public static PathPlannerTrajectory BlueTopIntakeToBalance = PathPlanner.loadPath("Blue Top Pick And Balance",
                        Constants.Drivebase.PATH_CONSTRAINTS);

        public static PathPlannerTrajectory RedTopIntakeToBalance = PathPlanner.loadPath("Blue Top Pick And Balance",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory ShiftAndPickBlue = PathPlanner.loadPath("Shift And Pick Blue",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory ShiftAndPickRed = PathPlanner.loadPath("Shift And Pick Red",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory RedBottomIntakeToBalance = PathPlanner.loadPath(
                        "Red Bottom Pick And Balance",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory BlueBottomIntakeToBalance = PathPlanner.loadPath(
                        "Blue Bottom Pick And Balance",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory ChargingOneCycleBlue = PathPlanner.loadPath(
                        "Charging One Cycle Blue",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory ChargingOneCycleRed = PathPlanner.loadPath(
                        "Charging One Cycle Red",
                        Constants.Drivebase.PATH_CONSTRAINTS);

        public static PathPlannerTrajectory RedTopBalance = PathPlanner.loadPath("Red Top Balance",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory MoveSideRed = PathPlanner.loadPath("Side Move Red",
                        Constants.Drivebase.PATH_CONSTRAINTS);
        public static PathPlannerTrajectory MoveSideBlue = PathPlanner.loadPath("Side Move Blue",
                        Constants.Drivebase.PATH_CONSTRAINTS);

        private Autos() {
                throw new UnsupportedOperationException("This is a utility class!");
        }
}
