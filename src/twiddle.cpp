#include "twiddle.h"

#include <limits>
#include <numeric>
#include <iostream>

using namespace std;

Twiddle::Twiddle(const std::vector<double>& p, const std::vector<double>& dp, double tolerance)
    : m_p(p)
    , m_dp(dp)
    , m_minError(std::numeric_limits<double>::max())
    , m_tolerance(tolerance)
    , m_currentIndex(p.size()-1)
    , m_up(true)
{

    // prepare dp for the first iteration where it will be multiplicated with 1.1
    m_dp[0] /= 1.1;
}

bool Twiddle::Next(std::vector<double>* p, double previousError) {

    if(previousError < m_minError) {
        m_minError = previousError;
        cout << "new minimum error e=" << previousError << " for p=[" << m_p[0];
        for(auto it = std::next(m_p.begin()); it != m_p.end(); ++it)
            cout << ", " << *it;
        cout << "]" << endl;

        m_dp[m_currentIndex] *= m_up ? 1.1 : 1.05;
        m_currentIndex = (m_currentIndex+1)%m_p.size();
        m_up = true;
    } else {
        cout << "twiddle e=" << previousError << " for p=[" << m_p[0];
        for(auto it = std::next(m_p.begin()); it != m_p.end(); ++it)
            cout << ", " << *it;
        cout << "]" << endl;

        if(m_up) {
            m_p[m_currentIndex] -= 3 * m_dp[m_currentIndex]; // substract 3 times instead of 2 because dp will be added in the end
            m_up = false;
        } else {
            m_p[m_currentIndex] += m_dp[m_currentIndex];
            m_dp[m_currentIndex] *= 0.95;
            m_currentIndex = (m_currentIndex+1)%m_p.size();
            m_up = true;
        }
    }

    m_p[m_currentIndex] += m_dp[m_currentIndex];
    p->resize(m_p.size());
    std::copy(m_p.begin(), m_p.end(), p->begin());

    if(accumulate(m_dp.begin(), m_dp.end(), 0.0) < m_tolerance)
        return false;

    return true;
}

