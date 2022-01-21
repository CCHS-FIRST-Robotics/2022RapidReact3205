package frc.robot.helper;

import java.lang.Math;

/**
 * Collection of all common mathematics operations with emphasis on vector math
 * 
 * @author Ludwig Tay
 */
public class SimpleMat {

    /**
     * Rotate a vector by theta radians and return resulting rotated vector. Done
     * using 2x2 rotation matrix
     * 
     * @param vec   len 2 array containing {x,y} values that represent vector to be
     *              rotated
     * @param theta angle to rotate vector by in radians
     * @return rotated vector
     */
    public static double[] rot2d(double[] vec, double theta) {
        double[] prod = { 0, 0 };
        prod[0] = Math.cos(theta) * vec[0] - Math.sin(theta) * vec[1];
        prod[1] = Math.sin(theta) * vec[0] + Math.cos(theta) * vec[1];
        return prod;
    }

    /**
     * Add 2 vectors a and b.
     * 
     * @param a len 2 array containing {x,y} values that represent vector
     * @param b len 2 array containing {x,y} values that represent vector
     * @return added vector
     */
    public static double[] add(double[] a, double[] b) {
        double[] o = { a[0] + b[0], a[1] + b[1] };
        return o;
    }

    /**
     * Subtract vector b from vector a
     * 
     * @param a len 2 array containing {x,y} values that represent vector
     * @param b len 2 array containing {x,y} values that represent vector
     * @return subtracted vector
     */
    public static double[] subtract(double[] a, double[] b) {
        double[] o = { a[0] - b[0], a[1] - b[1] };
        return o;
    }

    /**
     * Calculates euclidean magnitude of vector
     * 
     * @param vec len 2 array containing {x,y} values that represent vector
     * @return double of magnitude
     */
    public static double mag(double[] vec) {
        return Math.pow(vec[0] * vec[0] + vec[1] * vec[1], 0.5);
    }

    /**
     * Normalizes vector to its corresponding unit vector
     * 
     * @param vec len 2 array containing {x,y} values that represent vector
     * @return normalized magnitude 1 unit vector
     */
    public static double[] unitVec(double[] vec) {
        double mag = Math.pow(vec[0] * vec[0] + vec[1] * vec[1], 0.5) + 0.001;
        double[] unit = { vec[0] / mag, vec[1] / mag };
        return unit;
    }

    /**
     * Multiply vector by a scalar.
     * 
     * @param vec    len 2 array containing {x,y} values that represent vector
     * @param scalar scalar to multiply vector by
     * @return
     */
    public static double[] scaleVec(double[] vec, double scalar) {
        double[] newVec = { vec[0] * scalar, vec[1] * scalar };
        return newVec;
    }

    /**
     * Project a given vector on to the reference vector and measures the magnitude.
     * 
     * @param reference len 2 array containing {x,y} values that represent vector.
     * @param given     len 2 array containing {x,y} values that represent vector.
     * @return Magnitude of the projected vectors
     */
    public static double scalarProject(double[] reference, double[] given) {
        double[] unit_ref = unitVec(reference);
        double projection = unit_ref[0] * given[0] + unit_ref[1] * given[1];
        return projection;
    }

    /**
     * Project a vector of scalar magnitude in the direction of heading.
     * 
     * @param heading heading in radians. 0 Represents a line pointing straight up
     *                from origin.
     * @param scalar  magnitude of projected vector.
     * @return projected vector.
     */
    public static double[] projectHeading(double heading, double scalar) {
        double[] projected = { -1 * Math.sin(heading) * scalar, Math.cos(heading) * scalar };
        return projected;
    }

    /**
     * Takes theta and regularizes it to -pi to pi.
     * 
     * @param theta angle in radians [-inf, inf].
     * @return double normalized angle.
     */
    public static double angleRectifier(double theta) {
        double remainder = theta % (2 * Math.PI);
        if (remainder > Math.PI) {
            remainder = remainder - 2 * Math.PI;
        }
        return remainder;
    }

    /**
     * Dot product of vectors a and b.
     * 
     * @param a len 2 array containing {x,y} values that represent vector.
     * @param b len 2 array containing {x,y} values that represent vector.
     * @return a dot b vector
     */
    public static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1];
    }

    /**
     * DEPRECIATED DYSFUNCTIONAL Determine the heading of vector a
     * 
     * @param a len 2 array containing {x,y} values that represent vector.
     * @return theta angle in radians
     */
    public static double vec2theta(double[] a) {
        double[] unit = unitVec(a);
        double theta = Math.acos(unit[1]);
        if (unit[0] > 0) {
            theta = Math.PI * 2 - theta;
        }
        return theta;
    }

    /**
     * DEPRECIATED DYSFUNCTIONAL
     * 
     * @param a
     * @param b
     * @return
     */
    public static double vecsAngle(double[] a, double[] b) {
        double cos_val = dot(a, b) / (mag(a) * mag(b));
        double diff = Math.acos(cos_val);
        // determine if its left rot or right by calculating global vert theta
        double a_theta = vec2theta(a);
        double b_theta = vec2theta(b);

        double turn = b_theta - a_theta;

        double way1 = Math.abs(turn);
        double way2 = 2 * Math.PI - way1;
        if (way1 < way2) {
        } else {
            turn = turn * -1;
        }

        return turn;
    }

    /**
     * Calculate the smaller angle between heading and another vector y_p
     * 
     * @param heading heading in theta radians
     * @param y_p     len 2 array containing {x,y} values that represent vector.
     * @return The angle between heading and y_p in radians. (Positive: ccw,
     *         Negative: cw)
     */
    public static double vecsAngle2(double[] heading, double[] y_p) {
        double turn_mag = Math.acos(dot(heading, y_p) / (mag(heading) * mag(y_p) + 0.001));
        double[] new_head = { -1 * heading[1], heading[0] };
        double side = dot(new_head, y_p);
        double turn;
        if (side > 0) {
            turn = turn_mag;
        } else {
            turn = turn_mag * -1;
        }
        return turn;
    }

    /**
     * Calculates distance between vectors a and b
     * 
     * @param a len 2 array containing {x,y} values that represent vector.
     * @param b len 2 array containing {x,y} values that represent vector.
     * 
     * @return distance double.
     */
    public static double vectorDistance(double[] a, double[] b) {
        double[] new_vec = { a[0] - b[0], a[1] - b[1] };
        return mag(new_vec);
    }

    public static double unitClamp(double a) {
        return Math.min(1, Math.max(-1, a));
    }
}
