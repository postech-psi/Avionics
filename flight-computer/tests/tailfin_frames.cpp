#include "../src/m7/tailfin_controller.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace tailfin;

int main() {
    // Synthetic upright attitude with IMU +Z down, as required by legacy recovery.
    const auto imu = eulerDegrees(180,0,0);
    const Quaternion enuToNed{0,0.707106781f,0.707106781f,0};
    const Quaternion goal{0.707106781f,0,0.707106781f,0};
    auto mount = multiply(conjugate(multiply(enuToNed,imu)),goal);
    assert(normalize(mount));
    assert(std::fabs(2*(mount.x*mount.z-mount.w*mount.y)+1) < 1e-6f);

    Controller controller;
    const auto attitude = imuToSimulation(imu,mount);
    const float zeroRate[3]{};
    const auto neutral = controller.step(attitude,zeroRate,275,true);
    assert(neutral.enabled);
    for (float command : neutral.command) assert(std::fabs(command) < 1e-6f);

    const float imuRate[3]{0,0,-90};
    float bodyRate[3];
    gyroToSimulation(imuRate,mount,bodyRate);
    assert(std::fabs(bodyRate[0]-PI_F/2) < 1e-6f);
    assert(std::fabs(bodyRate[1])+std::fabs(bodyRate[2]) < 1e-6f);
    const auto delayed = controller.step(attitude,bodyRate,275,true,true,0.04f);
    assert(!delayed.enabled);
    for (float command : delayed.command) assert(commandToPulse(command,1500,1,0.0653f)==1500);
    std::puts("Tail-fin mounting, gyro and neutral checks passed");
}
