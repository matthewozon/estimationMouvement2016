#include "C_modelEstimation.h"

C_modelEstimation::C_modelEstimation(std::string fileNameFrame1, std::string fileNameFrame2, double frameDelay, int estimationMethod, double alpha, int N):C_motionEstimation(fileNameFrame1,fileNameFrame2,frameDelay)
{
    //compute the motion estimation
    if(estimationMethod==LAP)
    {
        computeMotionFieldLap();
    }
    else if(estimationMethod==V8)
    {
        computeMotionFieldV8();
    }
    else
    {
        computeMotionFieldHnS(alpha,N);
    }

    //set to the right size the parameters and label maps
    int _L=image1.getNbRow();
    int _C=image1.getNbRow();
    Au.resize(_L,_C);   Bu.resize(_L,_C);   Cu.resize(_L,_C);
    Av.resize(_L,_C);   Bv.resize(_L,_C);   Cv.resize(_L,_C);
    labelMap.resize(_L,_C);
}

C_imgMatrix<double> C_modelEstimation::applyAffineModel(double au, double bu, double cu, double av, double bv, double cv)
{
    C_imgMatrix<double> imAffine(image1.getNbRow(),image1.getNbColumn());
    imAffine=0.0;
    int lPrev,cPrev;
    for(int l=0 ; l<=imAffine.endL ; l++)
    {
        for(int c=0 ; c<=imAffine.endC ; c++)
        {
            //estimate position in the "next" frame
            lPrev=l-((int)(dt*(au*((double)l)+bu*((double)c)+cu)));
            cPrev=c-((int)(dt*(av*((double)l)+bv*((double)c)+cv)));
            if(cPrev>=0 && cPrev<=image1.endC && lPrev>=0 && lPrev<=image1.endL)
            {
                imAffine(l,c)=image1(lPrev,cPrev);
            }
        }
    }
    return imAffine;
}

bool C_modelEstimation::computeAfineEstimation(ROI R)
{
    return true;
}


bool C_modelEstimation::saveMaps(void)
{
    //u component
    std::ofstream myfile;
    myfile.open ("Au", std::ios::out ); //| std::ios::binary
    if(myfile.is_open())
    {
        for(unsigned short l=0 ; l<Au.getNbRow() ; l++)
        {
            for(unsigned short c=0 ; c<Au.getNbColumn() ; c++)
            {
                myfile << Au(l,c) << ", ";
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    else
    {
        return false;
    }


    myfile.open ("Bu", std::ios::out );
    if(myfile.is_open())
    {
        for(unsigned short l=0 ; l<Bu.getNbRow() ; l++)
        {
            for(unsigned short c=0 ; c<Bu.getNbColumn() ; c++)
            {
                myfile << Bu(l,c) << ", ";
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    else
    {
        return false;
    }

    myfile.open ("Cu", std::ios::out );
    if(myfile.is_open())
    {
        for(unsigned short l=0 ; l<Cu.getNbRow() ; l++)
        {
            for(unsigned short c=0 ; c<Cu.getNbColumn() ; c++)
            {
                myfile << Cu(l,c) << ", ";
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    else
    {
        return false;
    }




    //v component
    myfile.open ("Av", std::ios::out ); //| std::ios::binary
    if(myfile.is_open())
    {
        for(unsigned short l=0 ; l<Av.getNbRow() ; l++)
        {
            for(unsigned short c=0 ; c<Av.getNbColumn() ; c++)
            {
                myfile << Av(l,c) << ", ";
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    else
    {
        return false;
    }


    myfile.open ("Bv", std::ios::out );
    if(myfile.is_open())
    {
        for(unsigned short l=0 ; l<Bv.getNbRow() ; l++)
        {
            for(unsigned short c=0 ; c<Bv.getNbColumn() ; c++)
            {
                myfile << Bv(l,c) << ", ";
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    else
    {
        return false;
    }

    myfile.open ("Cv", std::ios::out );
    if(myfile.is_open())
    {
        for(unsigned short l=0 ; l<Cv.getNbRow() ; l++)
        {
            for(unsigned short c=0 ; c<Cv.getNbColumn() ; c++)
            {
                myfile << Cv(l,c) << ", ";
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    else
    {
        return false;
    }


    myfile.open ("labelMap", std::ios::out );
    if(myfile.is_open())
    {
        for(unsigned short l=0 ; l<labelMap.getNbRow() ; l++)
        {
            for(unsigned short c=0 ; c<labelMap.getNbColumn() ; c++)
            {
                myfile << labelMap(l,c) << ", ";
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    else
    {
        return false;
    }


    return true;
}
