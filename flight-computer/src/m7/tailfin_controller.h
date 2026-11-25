#pragma once
#include <cmath>

// Port of tailfincontrol_sim.slx chart_14 / controller_code.m, 2026-11-21.
// Pure math: no Arduino dependencies. All controller inputs use simulation NED
// and a model body frame with X pointing toward the nose. SI units except Euler/PWM.
// 계산 흐름: 현재 자세 -> 목표와의 회전 오차 -> 동압별 gain -> 요구 토크 -> 핀 각도.
// 모델의 gain/할당식을 그대로 옮겼으며, 서보 하드웨어 호출은 main_m7.cpp에서 담당한다.
namespace tailfin {
// Arduino defines PI as a macro; use a distinct name in this standalone header.
constexpr float PI_F = 3.14159265358979323846f;
constexpr float RAD = PI_F / 180.0f;
// 기본 계산 주기 10ms(100Hz). 실제 M7에서는 loop에서 측정한 dt를 전달한다.
constexpr float TS = 0.01f;
constexpr float LIMIT = 20.0f * RAD;
// gain 표의 열: 동압 50/150/400/1000/2000Pa. 행: 기체 roll/pitch/yaw 축.
// 각 표는 MATLAB의 기본 gain × configure_gain_schedule.m의 배율이다.
// Kp: N·m/rad, Ki: N·m/(rad·s), Kd: N·m/(rad/s).
constexpr float KNOTS[5] = {50, 150, 400, 1000, 2000};
constexpr float KP[3][5] = {
    {0.08f*.0875f,0.08f*.18f,0.08f*.35f,0.08f*.585f,0.08f*.80f},
    {1.55925f,2.156f,2.187f,1.62f,1.38f},
    {1.55925f,2.156f,2.187f,1.62f,1.38f}};
constexpr float KI[3][5] = {
    {.012f*.05f,.012f*.125f,.012f*.275f,.012f*.425f,.012f*.50f},
    {.08f*.1875f,.08f*.42f,.08f*.81f,.08f*1.08f,.08f},
    {.08f*.1875f,.08f*.42f,.08f*.81f,.08f*1.08f,.08f}};
constexpr float KD[3][5] = {
    {.045f*.2275f,.045f*.39f,.045f*.56f,.045f*.80f,.045f*.90f},
    {.784875f,1.10f,1.2768f,1.188f,1.05f},
    {.784875f,1.10f,1.2768f,1.188f,1.05f}};
// 스칼라 성분을 먼저 저장: (w,x,y,z). 기본값 (1,0,0,0)은 회전 없음.
struct Quaternion { float w=1, x=0, y=0, z=0; };
inline float clamp(float v,float lo,float hi) { return v<lo?lo:(v>hi?hi:v); }
// 쿼터니언 곱 a*b: 오른쪽 b의 회전을 먼저 적용한 뒤 a를 적용한다.
// 곱셈 순서를 바꾸면 일반적으로 다른 자세가 된다.
inline Quaternion multiply(const Quaternion& a,const Quaternion& b) {
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
            a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
            a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
// 길이가 1인 단위 쿼터니언으로 보정한다. NaN/무한대/영벡터는 사용하지 않는다.
inline bool normalize(Quaternion& q) {
    float n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    if (!std::isfinite(n) || n<1e-6f) return false;
    q.w/=n; q.x/=n; q.y/=n; q.z/=n; return true;
}
// IMU Euler 입력은 도(deg). q= qZ(yaw)*qY(pitch)*qX(roll) 순서로 변환한다.
// 쿼터니언 식에 반각이 들어가므로 각도에 RAD/2를 곱한다. 새 센서 융합은 아니다.
inline Quaternion eulerDegrees(float roll,float pitch,float yaw) {
    float cr=std::cos(roll*RAD/2),sr=std::sin(roll*RAD/2);
    float cp=std::cos(pitch*RAD/2),sp=std::sin(pitch*RAD/2);
    float cy=std::cos(yaw*RAD/2),sy=std::sin(yaw*RAD/2);
    return {cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
}
// qImu maps sensor vectors into ENU. imuFromBody maps model-body vectors
// into sensor coordinates and must describe the installed mounting.
inline Quaternion conjugate(const Quaternion& q) { return {q.w,-q.x,-q.y,-q.z}; }

inline Quaternion imuToSimulation(const Quaternion& qImu, const Quaternion& imuFromBody) {
    const Quaternion nedFromEnu={0,.7071067811865475f,.7071067811865475f,0};
    return multiply(multiply(nedFromEnu,qImu),imuFromBody);
}

inline void gyroToSimulation(const float dps[3], const Quaternion& imuFromBody, float radps[3]) {
    const Quaternion sensorRate={0,dps[0]*RAD,dps[1]*RAD,dps[2]*RAD};
    const Quaternion bodyRate=multiply(multiply(conjugate(imuFromBody),sensorRate),imuFromBody);
    radps[0]=bodyRate.x; radps[1]=bodyRate.y; radps[2]=bodyRate.z;
}

// 기존 UKF의 수직 속도로 동압 qbar=0.5*rho*vz^2를 근사한다.
// rho=1.225*exp(-h/8500): 모델과 같은 고도별 밀도 식, h는 0 이상으로 제한.
// 정압(hPa)이 아닌 동압(Pa)이며 바람·수평 속도를 포함한 대기속도 실측값은 아니다.
inline float verticalPressureEstimate(float altitudeM,float upwardMps) {
    if (!std::isfinite(altitudeM)||!std::isfinite(upwardMps)) return NAN;
    const float h=altitudeM>0?altitudeM:0;
    return .5f*1.225f*std::exp(-h/8500.0f)*upwardMps*upwardMps;
}
// 한 주기의 계산 결과. errorQuaternion은 회전 오차, error는 PID용 3축 오차다.
// demand는 토크(N·m), command는 네 핀의 명령 각도(rad), kp/ki/kd는 보간된 gain.
// attitudeValid는 수학적 쿼터니언 유효성이고, 최신 센서 여부까지 의미하지 않는다.
// enabled=false이면 command는 0으로 남아 PWM 변환 시 중립이 된다.
struct Result {
    Quaternion attitude;
    Quaternion errorQuaternion; // conjugate(current) * target, shortest rotation
    float kp[3]={0,0,0}, ki[3]={0,0,0}, kd[3]={0,0,0};
    float error[3]={0,0,0};
    float demand[3]={0,0,0}; // N*m
    float command[4]={0,0,0,0}; // nominal commanded fin angle, radians
    bool attitudeValid=false, enabled=false, saturated=false;
};
class Controller {
    // 주기 사이에 유지되는 축별 오차 적분값. 비활성화 때 reset()으로 지운다.
    float integral_[3]={0,0,0};
public:
    void reset() { for (float& v:integral_) v=0; }
    const float* integral() const { return integral_; }
    // q: 시뮬레이션 좌표계 자세, rate: 기체 각속도(rad/s), qbar: 동압(Pa).
    // scheduled=true는 gain 보간, false는 원래 고정 PID(대조 시험용).
    // main_m7.cpp는 true를 전달한다. dt의 단위는 초다.
    Result step(Quaternion q,const float rate[3],float qbar,bool enabled,
                bool scheduled=true,float dt=TS) {
        Result out;
        out.attitudeValid=normalize(q); out.attitude=q;
        if (!out.attitudeValid) { reset(); return out; }
        // 모델 목표 Euler [roll=0,pitch=90,yaw=0]의 쿼터니언.
        // 초기 85도를 강제하지 않는다. 매번 실제 센서 자세에서 이 목표까지 계산한다.
        const Quaternion target={.7071067811865475f,0,.7071067811865475f,0};
        // q_error = inverse(current) * target. 단위 쿼터니언의 역은 켤레다.
        // {w,-x,-y,-z}를 곱해 현재 기체 기준으로 필요한 회전을 구한다.
        Quaternion qe=multiply(conjugate(q),target);
        // q와 -q는 같은 회전. scalar가 양수인 쪽을 택해 짧은 회전 오차를 사용한다.
        const float sign=qe.w<0?-1.0f:1.0f;
        out.errorQuaternion={sign*qe.w,sign*qe.x,sign*qe.y,sign*qe.z};
        // 모델 그대로 error=2*q_error.xyz. 작은 오차에서는 rad 각도에 가깝지만
        // 정확한 axis-angle 벡터는 아니다. 5도 오차라면 2*sin(2.5도) ≈ 0.08724.
        out.error[0]=2*sign*qe.x; out.error[1]=2*sign*qe.y; out.error[2]=2*sign*qe.z;
        bool ratesOK=true; for (int a=0;a<3;++a) ratesOK &= std::isfinite(rate[a]);
        // Added firmware guards; nominal simulation parity uses dt=0.01.
        // 동압 50Pa 미만/비활성/비정상 입력/30ms 초과 지연이면 적분을 지우고 종료.
        // 유효하지 않은 이전 명령을 유지하지 않고 핀 각도 0(중립)을 반환한다.
        if (!enabled || !ratesOK || !std::isfinite(qbar) || qbar<50 ||
            !std::isfinite(dt) || dt<=0 || dt>0.03f) { reset(); return out; }
        // gain 선택에만 동압 범위를 제한한다. 구간 밖에서는 끝 gain을 유지한다.
        // 아래 핀 할당식의 K에는 원래 qbar를 사용하므로 gain clamp와 구분한다.
        float p=clamp(qbar,KNOTS[0],KNOTS[4]); int seg=0;
        while (seg<3 && p>KNOTS[seg+1]) ++seg;
        // 구간 내 비율 f: 아래 knot에서 0, 위 knot에서 1. 예: 275Pa는 150~400의 0.5.
        float f=(p-KNOTS[seg])/(KNOTS[seg+1]-KNOTS[seg]);
        const float baseP[3]={.08f,1,1},baseI[3]={.012f,.08f,.08f},baseD[3]={.045f,1,1};
        float candidate[3];
        for (int a=0;a<3;++a) {
            // 축마다 Kp/Ki/Kd를 각각 선형 보간: (1-f)*아래 gain + f*위 gain.
            float kp=scheduled?(1-f)*KP[a][seg]+f*KP[a][seg+1]:baseP[a];
            float ki=scheduled?(1-f)*KI[a][seg]+f*KI[a][seg+1]:baseI[a];
            float kd=scheduled?(1-f)*KD[a][seg]+f*KD[a][seg+1]:baseD[a];
            out.kp[a]=kp; out.ki[a]=ki; out.kd[a]=kd;
            // 이번 주기 적분 후보 I+error*dt를 ±0.25로 제한한다.
            // 실제 저장은 아래 포화 검사 후 수행한다(anti-windup).
            candidate[a]=clamp(integral_[a]+dt*out.error[a],-.25f,.25f);
            // 요구 토크 = P(자세 오차) + I(누적 오차) - D(현재 각속도).
            // D항은 오차 수치 미분이 아니라 모델과 동일한 자이로 감쇠 피드백이다.
            out.demand[a]=kp*out.error[a]+ki*candidate[a]-kd*rate[a];
        }
        // 모델의 핀 유효 계수: 동압 × 핀 면적 0.004m² × CN_delta 2.2.
        // 토크 -> 핀 각도 변환에는 flap_radius=0.09m, tail_arm=0.45m를 사용한다.
        const float K=qbar*.004f*2.2f;
        // 네 핀 방위각 0/90/180/270도의 cos와 sin을 미리 저장한다.
        const float c[4]={1,0,-1,0},s[4]={0,1,0,-1};
        float maxCmd=0;
        for (int i=0;i<4;++i) {
            // roll 토크는 네 핀에 공통 분배, pitch/yaw 토크는 배치각에 따라 분배.
            out.command[i]=out.demand[0]/(4*K*.090f)
                +c[i]*out.demand[1]/(2*K*.45f)+s[i]*out.demand[2]/(2*K*.45f);
            if (!std::isfinite(out.command[i])) { reset(); return Result{}; }
            if (std::fabs(out.command[i])>maxCmd) maxCmd=std::fabs(out.command[i]);
        }
        // 어느 핀이라도 ±20도를 넘으면 네 명령을 같은 비율로 줄인다.
        // 각 핀을 따로 잘라내는 방식보다 원래 토크 배분 비율을 유지한다.
        float scale=maxCmd>LIMIT?maxCmd/LIMIT:1;
        out.saturated=scale>1;
        for (float& v:out.command) v/=scale;
        // 포화되지 않았을 때만 적분 후보를 저장한다. 포화 중 적분 누적을 막는다.
        if (!out.saturated) for (int a=0;a<3;++a) integral_[a]=candidate[a];
        out.enabled=true; return out;
    }
};
// 핀 명령(rad) -> 도 -> 서보 펄스 폭(us). direction은 채널별 링크 방향 ±1.
// neutral + direction*command_deg/degPerUs; 이후 전기적 펄스 범위를 제한한다.
// 예: 중립 1500us, +20도, 0.0653deg/us이면 약 1806us.
// 서보의 50Hz 펄스 반복은 Servo 라이브러리가 담당하며 이 함수는 폭만 계산한다.
inline int commandToPulse(float radians,int neutral,int direction,float degPerUs,
                          int minUs=1000,int maxUs=2000) {
    if (!std::isfinite(radians)||!std::isfinite(degPerUs)||degPerUs<=0||
        (direction!=1&&direction!=-1)) return neutral;
    float us=neutral+direction*clamp(radians,-LIMIT,LIMIT)/RAD/degPerUs;
    return static_cast<int>(std::lround(clamp(us,float(minUs),float(maxUs))));
}
} // namespace tailfin
