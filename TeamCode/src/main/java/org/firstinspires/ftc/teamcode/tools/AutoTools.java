package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.List;

public class AutoTools {
    // List of obstacles on the red side
    public static final Obstacle RED_HIVE = new Obstacle(
            new ArrayList<>(List.of(new Pose(46, 50), new Pose(48.6, 50), new Pose(48.6, 91.4), new Pose(46, 91.4))));
    public static final Obstacle RED_ALLIANCE_FLOWER = new Obstacle(
            new ArrayList<>(List.of(new Pose(0, 50.35), new Pose(2.1, 50.35), new Pose(3.8, 47), new Pose(2.1, 43.65), new Pose(0, 43.65))));
    public static final Obstacle SCORING_SIDE_FLOWER = new Obstacle(
            new ArrayList<>(List.of(new Pose(43.4, 141.5), new Pose(50.1, 141.5), new Pose(50.1, 139.4), new Pose(46.75, 137.7), new Pose(43.4, 139.4))));




    public static Path buildPathLinearLine(Pose p1, Pose p2) { // This builds a path between two points, and gets used all the time in autos
        return Paths.line(p1, p2).linear(p1, p2);
    }

    public static boolean checkForCollision(Pose p1, Pose p2, Obstacle o) {
        return true;
    } // Not implemented

    public static class Obstacle {
        // An obstacle is a collection of vertices with the ability to tell you if a point is inside it
        private ArrayList<Pose> vertices;

        public Obstacle(ArrayList<Pose> v) {
            vertices = v;
        }

        public ArrayList<Pose> getVerts() {
            return vertices;
        }

        public boolean pointInBounds(Pose p) {
            // This was really annoying to make

            // Basically, it uses raycasting to the right of a single point.
            //  If there are an odd number of times is crosses the obstacles lines, it's inside the obstacle. Otherwise, its out
            if (vertices.size() < 3) return false;

            boolean inside = false;
            boolean intersect = ((vertices.get(vertices.size() - 1).y() > p.y()) != (vertices.get(0).y() > p.y()))
                    && p.x() < (vertices.get(0).x() - vertices.get(vertices.size() - 1).x()) * (p.y() - vertices.get((vertices.size() - 1)).y()) / (vertices.get(0).y() - vertices.get(vertices.size() - 1).y()) + vertices.get(vertices.size() - 1).x();

            if (intersect) {
                inside = !inside;
            }

            for (int i = 0; i < vertices.size() - 1; i++) {
                intersect = ((vertices.get(i).y() > p.y()) != (vertices.get(i + 1).y() > p.y()))
                        && p.x() < (vertices.get(i + 1).x() - vertices.get(i).x()) * (p.y() - vertices.get(i).y()) / (vertices.get(i + 1).y() - vertices.get(i).y()) + vertices.get(i).x();

                if (intersect) {
                    inside = !inside;
                }
            }

            return inside;
        }

    }
}
