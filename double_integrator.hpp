#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>


double bisection_newton(
    const std::function<double(double)>& f,
    const std::function<double(double)>& df,
    double start,
    double end,
    double tol = 0.05,
    int max_iter = 20
) {
    double est = (start + end) * 0.5;
    for (int i = 0; i < max_iter; ++i) {
        double fratio = f(est) / df(est);

        if ((end - (est - fratio)) * ((est - fratio) - start) < 0.0 || std::abs(fratio) < (end - start) / 4.0) {
            if (f(est) > 0) {
                end = est;
            } else {
                start = est;
            }
            fratio = est - (end + start) * 0.5;
        }
        est -= fratio;

        if (std::abs(fratio) < tol) {
            break;
        }
    }
    return est;
}

std::pair<double, double> optimal_cost(const Eigen::Vector4d& s0, const Eigen::Vector4d& s1) {
    Eigen::Vector2d x0 = s0.head<2>();
    Eigen::Vector2d v0 = s0.tail<2>();
    Eigen::Vector2d x1 = s1.head<2>();
    Eigen::Vector2d v1 = s1.tail<2>();
    Eigen::Vector2d x_diff = x1 - x0;
    Eigen::Vector2d v_diff = v1 - v0;

    double p = -4 * (v0.dot(v0) + v1.dot(v1) + v0.dot(v1));
    double q = 24 * (v0 + v1).dot(x_diff);
    double r = -36 * x_diff.dot(x_diff);

    auto cost = [&](double t) {
        return t + v_diff.dot(4.0 * v_diff / t - 6 * (-v0 * t + x_diff) / (t * t))
            + (-6 * v_diff / (t * t) + 12 * (x_diff - v0 * t) / (t * t * t)).dot(-v0 * t + x_diff);
    };

    auto cost_d1 = [&](double t) {
        return t * t * t * t + p * t * t + q * t + r;
    };

    auto cost_d2 = [&](double t) {
        return 4.0 * t * t * t + 2 * p * t + q;
    };

    double t_min = 0.0;
    double t_max = 10.0;
    double optimal_t = bisection_newton(cost_d1, cost_d2, t_min, t_max);
    return {cost(optimal_t), optimal_t};
}

