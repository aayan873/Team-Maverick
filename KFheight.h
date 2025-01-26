#include <MatrixMath.h>
class KFheight{
    private :
      double state[3]; // State vector
      double Q[3][3];     // Process noise covariance matrix
      double A[3][3];     // State transition matrix
      double R;           // Measurement noise covariance
      double P[3][3];     // State estimation error covariance matrix
      double K[3];        // Kalman gain 
      double H[1][3];     // Measurement matrix 
      double identity[3][3]; // Identity matrix 

      public :
        KFheight(double Qval, double R, double dt){

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Q[i][j] = (i == 0 && j == 0) ? Qval : 0.0; // Only the height component has process noise
                identity[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
          this->R = R;
          state[0] = 0.0; // Initial height
          state[1] = 0.0; // Initial velocity
          state[2] = 9.8; // Initial acceleration
          P[0][0] = P[1][1] = P[2][2] = 1.0; 
          P[0][1] = P[0][2] = P[1][0] = P[1][2] = P[2][0] = P[2][1] = 0.0;

          A[0][0] = 1.0; A[0][1] = dt; A[0][2] = 0.5*dt*dt;
          A[1][0] = 0.0; A[1][1] = 1.0; A[1][2] = dt;
          A[2][0] = 0.0; A[2][1] = 0.0; A[2][2] = 1.0;

          H[0][0] = 1.0; H[0][1] = 0.0; H[0][2] = 0.0; 

        }

        // Prediction step
        void predict(double paccn){
          // Update the accn in the state vector
          state[2] = paccn;

          double newState[3]; 
          Matrix.Multiply(&A[0][0], state, 3, 3, 1, newState);
          for (int i = 0; i < 3; i++) state[i] = newState[i];

          // Prediction of the new error covariance: P = A*P*A^T + Q
          double temp[3][3];
          Matrix.Multiply(&A[0][0], &P[0][0], 3, 3, 3, &temp[0][0]); // A*P
          double At[3][3];
          Matrix.Transpose(&A[0][0], 3, 3, &At[0][0]);   // Transpose of A
          Matrix.Multiply(&temp[0][0], &At[0][0], 3, 3, 3, &P[0][0]); // (A*P)*A^T
          Matrix.Add(&P[0][0], &Q[0][0], 3, 3, &P[0][0]);;  // adding Q
        }

        // Updation step
        void update(double mHeight) {
          double Ht[3][1];
          double P_Ht[3];
          double H_P_Ht[1][1];

          // Calculate Kalman gain: K = P*H^T/(H*P*H^T + R)
          Matrix.Transpose(&H[0][0], 1, 3, &Ht[0][0]);                  // H^T
          Matrix.Multiply(&P[0][0], &Ht[0][0], 3, 3, 1, P_Ht);          // P * H^T
          Matrix.Multiply(&H[0][0], P_Ht, 1, 3, 1, &H_P_Ht[0][0]);      // H * P * H^T
          double temp = H_P_Ht[0][0] + R;                               // Denom: H * P * H^T + R
          for (int i = 0; i < 3; i++) K[i] = P_Ht[i] / temp;            // Kalman gain: K = P * H^T / denom

          // Update state: state = state + K * (measurement - H * state)
          double r = mHeight - state[0]; // Residual: measurement - predicted height
          state[0] += K[0] * r; // Update height (state[0])

          // Update error covariance: P = (I - K * H) * P
          double KH[3][3], tempP[3][3];
          Matrix.Multiply(K, &H[0][0], 3, 1, 3, &KH[0][0]);             // K * H
          Matrix.Subtract(&identity[0][0], &KH[0][0], 3, 3, &tempP[0][0]); // I - K * H
          Matrix.Multiply(&tempP[0][0], &P[0][0], 3, 3, 3, &P[0][0]);   // (I - K * H) * P
        }
        // Get the estimated height
        double getHeight() const {
          return state[0];
        }
};