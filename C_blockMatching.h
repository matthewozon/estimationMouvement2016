#ifndef C_BLOCKMATCHING_H
#define C_BLOCKMATCHING_H

#include <C_imgMatrix.h>

struct mvtVec
{
    double dL;
    double dC;
};

class C_blockMatching
{
public:
    //ctor
    C_blockMatching(std::string fileNameFrame1, std::string fileNameFrame2);

    //predict the second frame using the parts of the first one
    C_imgMatrix<double> createNextFrame(int nbBlockL, int nbBlockC, int dL, int dC);

    //2 frames of a movie
    C_imgMatrix<double> image1;
    C_imgMatrix<double> image2;

private:

    //divide the first frame into nbL*nbC images
    std::vector<C_imgMatrix<double> > divideImage(int nbL, int nbC);

    //get the motion vector of the sub-image subImage searching in the region defined by block enlarged by dL and dC
    mvtVec getMotionVector(C_imgMatrix<double> subImage, int dL, int dC);


    /**********************************************************************************************/
    /******************************YOU SHALL CODE**************************************************/
    /**********************************************************************************************/
    //calculate a similarity function of subImage displaced by (dL,dC) considering the second frame
    double getBlockInterFrameDifference(C_imgMatrix<double> subImage, int dL, int dC);
    /**********************************************************************************************/
    /**********************************************************************************************/
    /**********************************************************************************************/


};

#endif // C_BLOCKMATCHING_H
