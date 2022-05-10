package org.firstinspires.ftc.teamcode.dog;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Leg {

    double curr_x = 0;
    double curr_y = 0;
    double curr_z = 0;
    ElapsedTime runtime = new ElapsedTime();

    double dist(Point a, Point b) {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    ArrayList<Point> circleIntersection(Point A, double r1, Point B, double r2) {
        double d = dist(A, B);

        if (d <= r1 + r2 && d >= abs(r2 - r1)) {

            double ex = (B.x - A.x) / d;
            double ey = (B.y - A.y) / d;

            double x = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
            double y = sqrt(r1 * r1 - x * x);

            Point P1 = new Point(A.x + x * ex - y * ey, A.y + x * ey + y * ex);
            Point P2 = new Point(A.x + x * ex + y * ey, A.y + x * ey - y * ex);

            ArrayList<Point> answer = new ArrayList<>();

            answer.add(P1);
            answer.add(P2);

            return answer;
        }

        return new ArrayList<Point>();

    }

    ArrayList<Double> getAngles(Point E) {
        Point A = new Point(0, 0);

        double d = 72.42;
        double d1 = 104.35;
        double d2 = 30;
        double d3 = 74.35;
        double d4 = 30;
        double d5 = 104.35;

        Point B = new Point(0, d);

        ArrayList<Point> inter = circleIntersection(A, d1, E, d5);

        if(inter.size() == 0) {
            return new ArrayList<>();
        }

        Point D = inter.get(0);

        double x = E.x + (D.x - E.x) / d5 * (d5 + d4);
        double y = E.y + (D.y - E.y) / d5 * (d5 + d4);

        Point C = new Point(x, y);

        inter = circleIntersection(B, d2, C, d3);

        if(inter.size() == 0) {
            return new ArrayList<>();
        }

        Point F = inter.get(1);

        double angle1 = asin(D.x / dist(A, D)) + (D.y < A.y ? PI : 0);
        double angle2 = asin(F.x / dist(B, F)) + (F.y < B.y ? PI : 0);

        ArrayList <Double> ans = new ArrayList<>();

        ans.add(angle1 / PI * 180);
        ans.add(angle2 / PI * 180);

        return ans;
    }

    Servo s0 = null;
    Servo s1 = null;
    Servo s2 = null;
    double coeff1, coeff2, coeff3, coeff4, coeff5, coeff6;

    Leg(HardwareMap hardwareMap, String s0_name, String s1_name, String s2_name,
        double coeff1, double coeff2, double coeff3, double coeff4, double coeff5, double coeff6) {

        s0 = hardwareMap.get(Servo.class, s0_name);
        s1 = hardwareMap.get(Servo.class, s1_name);
        s2 = hardwareMap.get(Servo.class, s2_name);

        s0.setPosition(coeff5);

        this.coeff1 = coeff1;
        this.coeff2 = coeff2;
        this.coeff3 = coeff3;
        this.coeff4 = coeff4;
        this.coeff5 = coeff5;
        this.coeff6 = coeff6;

        runtime.startTime();
    }

    void goTo(double x, double y) {

        ArrayList<Double> arr = getAngles(new Point(x, y));

        if(arr.size() > 0) {
            s1.setPosition(0.5 + (arr.get(1) - coeff1) / coeff2);
            s2.setPosition(0.5 + (arr.get(0) - coeff3) / coeff4);
            curr_x = x;
            curr_y = y;
        }
    }

    void goTo(double x, double y, double z) {

        x = sqrt(x * x + z * z);
        double angle = asin(z / x) / PI * 180;

        ArrayList<Double> arr = getAngles(new Point(x, y));

        if(arr.size() > 0) {
            s1.setPosition(0.5 + (arr.get(1) - coeff1) / coeff2);
            s2.setPosition(0.5 + (arr.get(0) - coeff3) / coeff4);
            curr_x = x;
            curr_y = y;
            curr_z = z;
            s0.setPosition(coeff5 + coeff6 * angle / 300);
        }

    }

    void interpolateTo(double x, double y, double milliseconds) {
        runtime.reset();

        double Curr_X = curr_x;
        double Curr_Y = curr_y;

        while(runtime.milliseconds() < milliseconds) {

            double ratio = runtime.milliseconds() / milliseconds;
            double X = Curr_X * (1 - ratio) + x * ratio;
            double Y = Curr_Y * (1 - ratio) + y * ratio;

            goTo(X, Y);

        }
    }

    void interpolateTo(double x, double y, double z, double milliseconds) {
        runtime.reset();

        double Curr_X = curr_x;
        double Curr_Y = curr_y;
        double Curr_Z = curr_z;

        while(runtime.milliseconds() < milliseconds) {

            double ratio = runtime.milliseconds() / milliseconds;
            double X = Curr_X * (1 - ratio) + x * ratio;
            double Y = Curr_Y * (1 - ratio) + y * ratio;
            double Z = Curr_Z * (1 - ratio) + z * ratio;

            goTo(X, Y, Z);

        }
    }

}