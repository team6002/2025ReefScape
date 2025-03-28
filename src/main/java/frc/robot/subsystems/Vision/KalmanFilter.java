package frc.robot.subsystems.Vision;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {
    private final int stateSize = 6;  // [x, y, theta, vx, vy, omega]
    private final int measurementSize = 3; // [x, y, theta]
    private final double dt = 0.04; // Time step

    private SimpleMatrix X;  // State vector
    private SimpleMatrix P;  // Covariance matrix
    private final SimpleMatrix Q;  // Process noise
    private final SimpleMatrix R;  // Measurement noise
    private final SimpleMatrix A;  // State transition matrix
    private final SimpleMatrix H;  // Measurement matrix

    public KalmanFilter() {
        // Initialize state [x, y, theta, vx, vy, omega]
        X = new SimpleMatrix(stateSize, 1);
        X.set(0, 1.0); // x
        X.set(1, 2.0); // y
        X.set(2, 0.1); // theta

        // Identity matrix for covariance
        P = SimpleMatrix.identity(stateSize).scale(0.1);

        // Process noise
        Q = SimpleMatrix.identity(stateSize).scale(0.01);

        // Measurement noise
        R = SimpleMatrix.identity(measurementSize).scale(0.05);

        // State transition matrix A
        A = SimpleMatrix.identity(stateSize);
        A.set(0, 3, dt); // x += vx * dt
        A.set(1, 4, dt); // y += vy * dt
        A.set(2, 5, dt); // theta += omega * dt

        // Measurement matrix H (we measure only x, y, theta)
        H = new SimpleMatrix(measurementSize, stateSize);
        for (int i = 0; i < measurementSize; i++) {
            H.set(i, i, 1);
        }
    }

    public void update(double[] measurement) {
        // Convert measurement array to a column vector SimpleMatrix
        SimpleMatrix Z = new SimpleMatrix(measurementSize, 1, true, measurement);

        // Prediction Step
        SimpleMatrix X_pred = A.mult(X);
        SimpleMatrix P_pred = A.mult(P).mult(A.transpose()).plus(Q);

        // Kalman Gain
        SimpleMatrix S = H.mult(P_pred).mult(H.transpose()).plus(R);
        SimpleMatrix K = P_pred.mult(H.transpose()).mult(S.invert());

        // Update Step
        SimpleMatrix Y = Z.minus(H.mult(X_pred));
        X = X_pred.plus(K.mult(Y));
        P = (SimpleMatrix.identity(stateSize).minus(K.mult(H))).mult(P_pred);
    }

    public void printState() {
        System.out.println("Updated State:");
        X.print();
    }

    public SimpleMatrix getState() {
        // System.out.println("Updated State:");
        return X;
    }

    public static void main(String[] args) {
        KalmanFilter kf = new KalmanFilter();

        // Simulated measurement (x, y, theta)
        double[] Z = {1.2, 2.1, 0.12};

        kf.update(Z);
        kf.printState();
    }
}
