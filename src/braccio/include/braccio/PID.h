#ifndef PID_H_
#define PID_H_

struct PID {
  private:
    float P, I, D;
    float cummulate_error, rate_error;
    short last_error;

  public:
    void Init() {
      last_error = 0;
      rate_error = 0;
      cummulate_error = 0;
    }

    float Compute(short error, const short divider) {
      if (error > divider)
        error = divider;
      else if (error < -divider)
        error = -divider;

      cummulate_error += error / divider;
      rate_error = (error - last_error) / divider;

      P = KP * error;
      I = KI * cummulate_error;
      D = KD * rate_error;

      last_error = error;

      return (P + I + D);
    }

    float KP, KI, KD;
};

#endif // PID_H_