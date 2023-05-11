#include <iostream>
#include <cmath>
#include <vector>

struct QuadState {
    double x, y, z;
    double qw, qx, qy, qz;
};

struct QuadMotor {
    double x, y, z;
};

class QuadPlot {
public:
    int k;
    int qn;
    double time;
    QuadState state;
    QuadMotor motor;

    std::vector<std::vector<double>> state_hist;
    std::vector<double> time_hist;

    QuadPlot(int qn, const QuadState& state, double wingspan, double height, int max_iter)
        : qn(qn), state(state), time(0), k(0)
    {
        motor.x = 0;
        motor.y = 0;
        motor.z = height;
        state_hist.resize(6, std::vector<double>(max_iter));
        time_hist.resize(max_iter);
    }

    void UpdateQuadState(const QuadState& state, double time)
    {
        this->state = state;
        this->time = time;
    }

    void UpdateQuadHist()
        state_hist[0][k] = state.x;
        state_hist[1][k] = state.y;
        state_hist[2][k] = state.z;
        state_hist[3][k] = state.qw;
        state_hist[4][k] = state.qx;
        state_hist[5][k] = state.qy;
        time_hist[k] = time;
        k++;
    }

    void UpdateMotorPos()
    {
        double L = motor.x; // Length of the quad
        double H = motor.z; // Height of the quad

        // Calculate coordinates of quadrotor's position in world frame
        double qw = state.qw, qx = state.qx, qy = state.qy, qz = state.qz;
        double x = state.x, y = state.y, z = state.z;

        double qahat[3][3] = {
            { 0, -qz,  qy},
            { qz,  0, -qx},
            {-qy,  qx,  0}
        };

        double R[3][3] = {
            {1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw},
            {2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw},
            {2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy}
        };

        motor.x = R[0][0]*L + x;
        motor.y = R[1][0]*L + y;
        motor.z = R[2][0]*L + z + H;
    }

    void UpdateQuadPlot(const QuadState& state, double time)
    {
        UpdateQuadState(state, time);
        UpdateQuadHist();
        UpdateMotorPos();
        // Update quad plot
        // ...
    }
};

int main() {
    // Usage example
    QuadState state;
    state.x = 0;
    state.y = 0;
    state.z = 0;
    state.qw = 1;
