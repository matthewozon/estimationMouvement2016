#ifndef C_MODELESTIMATION_H
#define C_MODELESTIMATION_H

#include "C_motionEstimation.h"

class ROI //Region Of Interest
{
public:
    ROI(){N=0;dx=0.0, dy=0.0;}
    ROI(double _dx, double _dy)
    {
        dx=_dx;
        dy=_dy;
        N=0;
    }

    //indices of the pixels in the ROI
    std::vector<int> idxL;
    std::vector<int> idxC;
    //location of the pixel (probably just a cast of the indices)
    std::vector<double> x;
    std::vector<double> y;
    //number of pixels in the ROI
    int N;
    //pixel spacing
    double dx, dy;
};


class C_modelEstimation : public C_motionEstimation
{
public:
    C_modelEstimation(std::string fileNameFrame1, std::string fileNameFrame2, double frameDelay, int estimationMethod=LAP, /*if method is HnS->*/double alpha=0.0, int N=0.0);

    //create data applying the affine model on the first image
    C_imgMatrix<double> applyAffineModel(double au, double bu, double cu, double av, double bv, double cv);

    //method that implements the motion estimation using an afine model considering a given region
    bool computeAfineEstimation(ROI R);

    //parameter of the model
    C_imgMatrix<double> Au;
    C_imgMatrix<double> Bu;
    C_imgMatrix<double> Cu;

    C_imgMatrix<double> Av;
    C_imgMatrix<double> Bv;
    C_imgMatrix<double> Cv;

    C_imgMatrix<int> labelMap;

    //save all maps
    bool saveMaps(void);

    //method that could be implemented: a partitionning of the image based on the model parameters -> classification
};

#endif // C_MODELESTIMATION_H
