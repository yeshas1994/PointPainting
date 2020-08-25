#pragma once

namespace DosUtils
{

// 1D Kalman Filter, Complete Model with Process Noise
class KalmanScalarFilter
{
public:
    KalmanScalarFilter() = default;
    KalmanScalarFilter(float startVal, float errVariance, float noiseVariance, float measureError);

    virtual ~KalmanScalarFilter() {}

    float run(float measureVal);
    float getCurVal() const;

protected:
    float m_curVal = 10.0f;
    float m_errVariance = 100000.0f;
    float m_noiseVariance = 0.0001f;
    float m_measureError = 0.1f;
};

} // end of namespace DosUtils