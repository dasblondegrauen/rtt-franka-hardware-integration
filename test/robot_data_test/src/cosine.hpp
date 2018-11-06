#ifndef COSINE_HPP
#define COSINE_HPP

#include <eigen3/Eigen/Core>

template <class T>
class Cosine
{
public:
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

    Cosine() {}
    Cosine(Vector initial_conf, Vector amplitude, double period)
        : initial_conf(initial_conf),
          amplitude(amplitude),
          period(period) {
        frequency = 2.0 * M_PI / this->period;
        ret = Vector(this->initial_conf);
    }

    Vector getQ(const double time) {
        ret = -0.5 * amplitude * (std::cos(frequency*time) - 1) + initial_conf;
        return ret;
    }

private:
    double period, frequency;
    Vector initial_conf, amplitude;
    Vector ret;
};

#endif // COSINE_HPP
