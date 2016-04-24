#include "C_motionEstimation.h"

C_motionEstimation::C_motionEstimation(std::string fileNameFrame1, std::string fileNameFrame2, double frameDelay)
{
    image1.load(fileNameFrame1);
    image2.load(fileNameFrame2);
    //image1.display(NO_NORMALIZATION);
    //image2.display(NO_NORMALIZATION);

    //compute derivatives
    dt = frameDelay;
    Ix = image1.gradX();
    Iy = image1.gradY();
    It = (image2 - image1)*(1.0/dt);
}

bool C_motionEstimation::computeMotionFieldLap(void)
{
    //initialize vector field to the null field
    vectX.resize(image1.getNbRow(),image1.getNbColumn());
    vectX=0.0;
    vectY.resize(image1.getNbRow(),image1.getNbColumn());
    vectY=0.0;

    //compute laplacian of image
    C_matrix<double> h(3,3);
    h(0,0) = 1.0; h(0,1) = 1.0; h(0,2) = 1.0;
    h(1,0) = 1.0; h(1,1) = -8.0; h(1,2) = 1.0;
    h(2,0) = 1.0; h(2,1) = 1.0; h(2,2) = 1.0;
    C_imgMatrix<double> imLap1 = image1.conv2(h);
    C_imgMatrix<double> imLap2 = image2.conv2(h);
    C_imgMatrix<double> Lx = imLap1.gradX();
    C_imgMatrix<double> Ly = imLap1.gradY();
    C_imgMatrix<double> Lt = (imLap2 - imLap1)*(1.0/dt);

    //for all pixels but on the boundaries
    C_matrix<double> A(2,2);
    C_matrix<double> B(2,1);
    C_matrix<double> C(2,1);
    for(int x=1 ; x<(image1.getNbColumn()-1) ; x++)
    {
        for(int y=1 ; y<(image1.getNbRow()-1) ; y++)
        {
            //build system to solve
            //current pixel
            A(0,0) = Ix(y,x); A(0,1) = Iy(y,x); B(0,0) = -It(y,x);
            A(1,0) = Lx(y,x); A(1,1) = Ly(y,x); B(1,0) = -Lt(y,x);

            //solve systeme if possible
            if(ABS(A.CalcDeterminant())>SMALL_NUM_D)
            {
                C = A.inv()*B;

                //store componant
                vectX(y,x) = C(0,0);
                vectY(y,x) = C(1,0);
            }
            else
            {
                //set to the null vector
                vectX(y,x) = 0.0;
                vectY(y,x) = 0.0;
            }

        }
    }

    vectX.display(NORMALIZE);
    vectY.display(NORMALIZE);

    //we should handle boundaires... but only if we have time. Anyway, it does not make much sens to handle those cases

    return true;
}

bool C_motionEstimation::computeMotionFieldV8(void)
{
    //initialize vector field to the null field
    vectX.resize(image1.getNbRow(),image1.getNbColumn());
    vectX=0.0;
    vectY.resize(image1.getNbRow(),image1.getNbColumn());
    vectY=0.0;

    //for all pixels but on the boundaries
    for(int x=1 ; x<(image1.getNbColumn()-1) ; x++)
    {
        for(int y=1 ; y<(image1.getNbRow()-1) ; y++)
        {
            //build system to solve

            //solve system

            //store componant
        }
    }

    vectX.display(NORMALIZE);
    vectY.display(NORMALIZE);

    //we should handle boundaires... but only if we have time. Anyway, it does not make much sens to handle those cases

    return true;
}


bool C_motionEstimation::computeMotionFieldHnS(double alpha, unsigned long nbIteration)
{
    //init vector field

    //iteration
    C_matrix<double> U(2,1);
    for(int i=0 ; i<nbIteration ; i++)
    {
        //for each pixel
        for(int x=1 ; x<(image1.getNbColumn()-1) ; x++)
        {
            for(int y=1 ; y<(image1.getNbRow()-1) ; y++)
            {
                //compute average of neighboring vector field

                //compute iteration
            }
        }
    }
    return true;
}




bool C_motionEstimation::saveVectorField(std::string fileNameVectX, std::string fileNameVectY)
{
    std::ofstream myfile;
    myfile.open (fileNameVectX.data(), std::ios::out ); //| std::ios::binary
    if(myfile.is_open())
    {
        for(unsigned short l=0 ; l<vectX.getNbRow() ; l++)
        {
            for(unsigned short c=0 ; c<vectX.getNbColumn() ; c++)
            {
                myfile << vectX(l,c) << ", ";
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    else
    {
        return false;
    }


    std::ofstream myfile2;
    myfile2.open (fileNameVectY.data(), std::ios::out );
    if(myfile2.is_open())
    {
        for(unsigned short l=0 ; l<vectY.getNbRow() ; l++)
        {
            for(unsigned short c=0 ; c<vectY.getNbColumn() ; c++)
            {
                myfile2 << vectY(l,c) << ", ";
            }
            myfile2 << std::endl;
        }
        myfile2.close();
    }
    else
    {
        return false;
    }
    return true;
}
