//
// Created by cc on 20/05/22.
//

#ifndef BOTBUSTERS_REBIRTH_RATELIMITER_H
#define BOTBUSTERS_REBIRTH_RATELIMITER_H

#include <wpi/timestamp.h>
#include <units/time.h>
#include <algorithm>


class RateLimiter {
public:
    explicit RateLimiter(double rateLimit, double initialVal = double{0}){
        m_prevVal = initialVal;
        m_rateLimit = rateLimit;
        m_rateModifier = 1;
        m_prevTime = units::microsecond_t (wpi::Now());
    }
//
    void setRateLimit(double rateLimit){
        m_rateLimit = rateLimit;
    }

    void setRateModifier(double modifier) { m_rateModifier = modifier;}

    [[nodiscard]] double getRateLimit() const{ return m_rateLimit; }

    double calculate(double input) {
        m_rateLimit *= m_rateModifier;
        units::second_t currentTime = units::microsecond_t(wpi::Now());
        units::second_t elapsedTime = currentTime - m_prevTime;
        m_prevVal += std::clamp(input - m_prevVal, -m_rateLimit * elapsedTime.value(), m_rateLimit * elapsedTime.value());
        m_prevTime = currentTime;
        return m_prevVal;
    }

    void reset(double value) {
        m_prevVal = value;
        m_prevTime = units::microsecond_t(wpi::Now());
    }



private:
    double m_rateLimit{};
    double m_prevVal{};
    units::second_t m_prevTime{};
    double m_rateModifier{};
};


#endif //BOTBUSTERS_REBIRTH_RATELIMITER_H
