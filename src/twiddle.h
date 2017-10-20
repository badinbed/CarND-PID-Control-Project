#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

class Twiddle
{
public:
    Twiddle(const std::vector<double>& p, const std::vector<double>& dp, double tolerance);

    bool Next(std::vector<double> *p, double previousError);
    double MinimumError() const { return m_minError; }

private:
    std::vector<double> m_p;
    std::vector<double> m_dp;
    double m_minError;
    double m_tolerance;
    std::size_t m_currentIndex;
    bool m_up;
};

#endif // TWIDDLE_H
