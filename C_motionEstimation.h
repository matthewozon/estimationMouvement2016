#ifndef C_MOTIONESTIMATION_H
#define C_MOTIONESTIMATION_H

#include <C_imgMatrix.h>

#define LAP 0
#define V8  1
#define HNS 2

class C_motionEstimation
{
public:
    //ctor: load two images (frames of a movie)
    C_motionEstimation(std::string fileNameFrame1, std::string fileNameFrame2, double frameDelay);

    /**********************************************************************************************/
    /******************************YOU SHALL CODE**************************************************/
    /**********************************************************************************************/
    //motion estimation using the 8-neighboring systeme (assumption: motion field is locally constant)
    bool computeMotionFieldV8(void);

    //method that computes all iterations of Horn & Schunck method: stores the result in vectX & vectY
    bool computeMotionFieldHnS(double alpha, unsigned long nbIteration);

    /**********************************************************************************************/
    /**********************************************************************************************/
    /**********************************************************************************************/



    /**********************************************************************************************/
    /******************already implemented, you should get inspiration here************************/
    /**********************************************************************************************/
    //determine the motion field using the original image and its laplacian
    bool computeMotionFieldLap(void);

    //the two frames
    C_imgMatrix<double> image1;
    C_imgMatrix<double> image2;

    //the vector field: flow vector
    C_imgMatrix<double> vectX;
    C_imgMatrix<double> vectY;

    //tool to save the vector field (may be useful to export result so that you can plot it using octave)
    bool saveVectorField(std::string fileNameVectX, std::string fileNameVectY);

protected:
    //first order derivatives of the images
    C_imgMatrix<double> Ix;
    C_imgMatrix<double> Iy;
    C_imgMatrix<double> It;

    //frame delay
    double dt;

};

#endif // C_MOTIONESTIMATION_H
