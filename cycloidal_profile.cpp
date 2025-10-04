#include <iostream>
#include <vector>
#include <cmath>

struct Point {
    double x, y;
};

std::vector<Point> generate_cycloidal_profile(double overall_diameter, int number_of_pins, double ecc_factor) {
    double pin_fraction = 0.15;
    double pin_diameter = pin_fraction * overall_diameter;
    double pin_circle_radius = (overall_diameter - pin_diameter) / 2.0;
    int samples = 3000;
    double rolling_circle_radius = pin_circle_radius / number_of_pins;
    double R = (number_of_pins - 1) * rolling_circle_radius;
    double E = ecc_factor * rolling_circle_radius;
    double Rr = pin_diameter / 2.0;

    std::vector<Point> full_pts;
    for (int i = 0; i < samples; ++i) {
        double t = (-30.0 + 240.0 * i / (samples - 1)) * M_PI / 180.0;
        int N = number_of_pins;
        int m = 1 - N;
        double denom = (R / (E * N)) - std::cos(m * t);
        if (std::abs(denom) < 1e-9) denom += (denom >= 0 ? 1e-9 : -1e-9);
        double ang = std::atan2(std::sin(m * t), denom);
        double x = R * std::cos(t) - Rr * std::cos(t + ang) - E * std::cos(N * t);
        double y = -R * std::sin(t) + Rr * std::sin(t + ang) + E * std::sin(N * t);
        full_pts.push_back({x, y});
    }
    return full_pts;
}

int main() {
    double overall_diameter = 150.0;
    int number_of_pins = 7;
    double ecc_factor = 0.6;

    std::vector<Point> pts = generate_cycloidal_profile(overall_diameter, number_of_pins, ecc_factor);

    std::cout << "Cycloidal Rotor Profile Points:\n";
    for (const auto& pt : pts) {
        std::cout << pt.x << ", " << pt.y << std::endl;
    }
    return 0;
}
