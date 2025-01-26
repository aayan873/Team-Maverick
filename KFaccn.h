#ifndef KFACCN_H
#define KFACCN_H

class KFaccn{
    private :
      float Q;  // Process noise covariance
      float R;  // Measurement noise covariance
      float x;  // State estimate
      float P;  // Estimation error covariance
      float K;  // Kalman gain 

      public :
        KFaccn(float Q, float R, float P){
          this->Q = Q;
          this->R = R;
          this->P = P;
          x = 9.8; //initial acceleration estimation
          K = 0;  //initial kalman gain
        }

        void update(float m){
          P += Q;
          K = P/(P + R);
          x += K*(m - x);
          P *= (1-K);
        }  
        float getState() const {
          return x;
    }
};
#endif