#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

// 定义路径格式
typedef std::vector<Eigen::Vector2d> Path;

// 轨迹段
struct TrajectorySegment {
    double t0; // 起始时间
    double tf; // 结束时间
    Eigen::Vector2d start_pos;
    Eigen::Vector2d end_pos;
    Eigen::Vector2d start_vel;
    Eigen::Vector2d end_vel;
    Eigen::Vector2d start_acc;
    Eigen::Vector2d end_acc;
    Eigen::MatrixXd coeffs_x; // 多项式系数
    Eigen::MatrixXd coeffs_y;
};

// 计算五次多项式系数
Eigen::VectorXd computeQuinticCoeffs(double t0, double tf, double p0, double pf, double v0, double vf, double a0, double af){
    double T = tf - t0;
    Eigen::MatrixXd A(3,3);
    A <<  T*T*T,    T*T*T*T,     T*T*T*T*T,
          3*T*T,  4*T*T*T,   5*T*T*T*T,
          6*T,   12*T*T,    20*T*T*T;

    Eigen::VectorXd B;
    B << pf - (p0 + v0*T + 0.5*a0*T*T),
         vf - (v0 + a0*T),
         af - a0;

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(B);
    Eigen::VectorXd coeffs(6);
    coeffs << p0, v0, 0.5*a0, x(0), x(1), x(2);
    return coeffs;
}

// 轨迹生成器类
class TrajectoryGenerator {
public:
    TrajectoryGenerator() = default;

    // 生成轨迹
    std::vector<Eigen::Vector2d> generateTrajectory(const Path& path, double total_time){
        if(path.empty()){
            std::cerr << "Empty path provided!" << std::endl;
            return {};
        }

        size_t num_segments = path.size() - 1;
        double dt = total_time / num_segments;

        std::vector<TrajectorySegment> segments;
        std::vector<Eigen::Vector2d> trajectory;

        // 初始化初始状态
        Eigen::Vector2d prev_vel(0.0, 0.0);
        Eigen::Vector2d prev_acc(0.0, 0.0);

        for(size_t i = 0; i < num_segments; ++i){
            TrajectorySegment seg;
            seg.t0 = i * dt;
            seg.tf = (i + 1) * dt;
            seg.start_pos = path[i];
            seg.end_pos = path[i+1];
            seg.start_vel = prev_vel;
            seg.start_acc = prev_acc;

            // 如果是最后一个轨迹段，设置终点速度和加速度为0
            if(i == num_segments - 1){
                seg.end_vel = Eigen::Vector2d(0.0, 0.0);
                seg.end_acc = Eigen::Vector2d(0.0, 0.0);
            } else {
                // 为了保持连续性，设置终点速度和加速度为下一段的起点速度和加速度
                // 这里可以根据需要设置，比如使用一个平滑的速度过渡
                // 目前简化为保持上一段的终点速度和加速度
                seg.end_vel = prev_vel; // 需要根据具体情况调整
                seg.end_acc = prev_acc; // 需要根据具体情况调整
            }

            // 计算x方向的多项式系数
            seg.coeffs_x = computeQuinticCoeffs(seg.t0, seg.tf, 
                                               seg.start_pos.x(), seg.end_pos.x(), 
                                               seg.start_vel.x(), seg.end_vel.x(),
                                               seg.start_acc.x(), seg.end_acc.x());

            // 计算y方向的多项式系数
            seg.coeffs_y = computeQuinticCoeffs(seg.t0, seg.tf, 
                                               seg.start_pos.y(), seg.end_pos.y(), 
                                               seg.start_vel.y(), seg.end_vel.y(),
                                               seg.start_acc.y(), seg.end_acc.y());

            // 更新下一段的初始速度和加速度为当前段的终点速度和加速度
            prev_vel = seg.end_vel;
            prev_acc = seg.end_acc;

            segments.push_back(seg);
        }

        // 采样轨迹
        double sampling_dt = 0.01; // 100 Hz
        for(double t = 0.0; t <= total_time; t += sampling_dt){
            // 找到当前时间所在的轨迹段
            size_t seg_idx = std::min(static_cast<size_t>(t / dt), segments.size()-1);
            const auto& seg = segments[seg_idx];
            double tau = t - seg.t0;

            // 计算位置
            double pos_x = seg.coeffs_x(0) + seg.coeffs_x(1)*tau + seg.coeffs_x(2)*tau*tau +
                           seg.coeffs_x(3)*pow(tau,3) + seg.coeffs_x(4)*pow(tau,4) + seg.coeffs_x(5)*pow(tau,5);
            double pos_y = seg.coeffs_y(0) + seg.coeffs_y(1)*tau + seg.coeffs_y(2)*tau*tau +
                           seg.coeffs_y(3)*pow(tau,3) + seg.coeffs_y(4)*pow(tau,4) + seg.coeffs_y(5)*pow(tau,5);

            trajectory.emplace_back(pos_x, pos_y);
        }

        return trajectory;
    }

private:
    // 计算五次多项式系数（辅助函数）
    Eigen::VectorXd computeQuinticCoeffs(double t0, double tf, double p0, double pf, double v0, double vf, double a0, double af){
        double T = tf - t0;
        Eigen::MatrixXd A(3,3);
        A <<  T*T*T,    T*T*T*T,     T*T*T*T*T,
              3*T*T,  4*T*T*T,   5*T*T*T*T,
              6*T,   12*T*T,    20*T*T*T;

        Eigen::VectorXd B;
        B << pf - (p0 + v0*T + 0.5*a0*T*T),
             vf - (v0 + a0*T),
             af - a0;

        Eigen::VectorXd x = A.colPivHouseholderQr().solve(B);
        Eigen::VectorXd coeffs(6);
        coeffs << p0, v0, 0.5*a0, x(0), x(1), x(2);
        return coeffs;
    }
};



int main(){
    // path 是从 A* 算法获得的路径
    Path path = {
        Eigen::Vector2d(-4.5, -4.5),
        Eigen::Vector2d(-3.0, -3.0),
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(3.0, 3.0),
        Eigen::Vector2d(4.5, 4.5)
    };

    double total_time = 10.0; // 总时间10秒

    TrajectoryGenerator traj_gen;
    std::vector<Eigen::Vector2d> trajectory = traj_gen.generateTrajectory(path, total_time);

    // 输出轨迹点
    for(const auto& point : trajectory){
        std::cout << "x: " << point.x() << ", y: " << point.y() << std::endl;
    }

    return 0;
}
