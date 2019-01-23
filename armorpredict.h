#ifndef ARMORPREDICT_H
#define ARMORPREDICT_H
#include "Header.h"
#include "serialport.h"
#include "solvepnp.h"
#include "stereo_vision.h"

static inline bool PosSort(const AbsPosition a1,const AbsPosition a2){
    return a1.z < a2.z;
}

class ArmorPredict
{
public:
    ArmorPredict();
    int Predict(vector<AbsPosition> Positions);

private:
    void AngleFit( AbsPosition& input);
    void Fresh();
public:
    AbsPosition Result, OldResult;
    VisionData Vision;
private:
    vector<AbsPosition> OldPositions;
    float yaw_out,pitch_out;
    float yaw_old,pitch_old;
    float shoot_speed;
};

#endif // ARMORPREDICT_H
