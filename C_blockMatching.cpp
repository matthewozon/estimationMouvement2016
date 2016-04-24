#include "C_blockMatching.h"

C_blockMatching::C_blockMatching(std::string fileNameFrame1, std::string fileNameFrame2)
{
    image1.load(fileNameFrame1);
    image2.load(fileNameFrame2);
}

double C_blockMatching::getBlockInterFrameDifference(C_imgMatrix<double> subImage /*block of the first frame*/, int dL, int dC)
{
    //check boudaries
    int lbegin, lend, cbegin, cend; //image limit without shift
    int subLbegin, subLend, subCbegin, subCend; //subimage limit

    if( (subImage.offsetL+dL)<0)
    {
        lbegin=0;
        subLbegin = -dL;
    }
    else
    {
        lbegin = subImage.offsetL+dL;
        subLbegin = 0;
    }

    if( (subImage.offsetC+dC)<0)
    {
        cbegin=0;
        subCbegin = -dC;
    }
    else
    {
        cbegin = subImage.offsetC+dC;
        subCbegin = 0;
    }



    if( (subImage.offsetL+subImage.getNbRow()+dL)>image2.endL)
    {
        lend = image2.endL;
        subLend = subImage.endL-dL;
    }
    else
    {
        lend = subImage.offsetL+subImage.getNbRow()+dL;
        subLend = subImage.endL;
    }

    if( (subImage.offsetC+subImage.getNbColumn()+dC)>image2.endC)
    {
        cend = image2.endC;
        subCend = subImage.endC-dC;
    }
    else
    {
        cend = subImage.offsetC+subImage.getNbColumn()+dC;
        subCend = subImage.endC;
    }

    //compute the similarity function.
    double E = 0.0;

    //return the value of similarity function
    return E;
}


std::vector<C_imgMatrix<double> > C_blockMatching::divideImage(int nbL, int nbC)
{
    std::vector<C_imgMatrix<double> > subImage;

    //create nbL*nbC blocks from the first frame
    for(int i=0 ; i<nbL ; i++)
    {
        for(int j=0 ; j<nbC ; j++)
        {
            try
            {
                //check boundaries
                int mL, mC;
                if( (((i+1)*image1.getNbRow())/nbL)>=image1.getNbRow())
                {
                    mL=image1.endL;
                }
                else
                {
                    mL = (((i+1)*image1.getNbRow())/nbL);
                }

                if( (((j+1)*image1.getNbColumn())/nbC)>=image1.getNbColumn())
                {
                    mC=image1.endC;
                }
                else
                {
                    mC = (((j+1)*image1.getNbColumn())/nbC);
                }

                //copy a block and push it into the vector
                subImage.push_back(image1.subset((i*image1.getNbRow())/nbL,mL,(j*image1.getNbColumn())/nbC,mC));
                //don't forget to update the offset (position of the upper left corner of the block in the image)
                subImage.back().offsetL = (i*image1.getNbRow())/nbL;
                subImage.back().offsetC = (j*image1.getNbColumn())/nbC;
            }
            catch(char const* a)
            {
                std::cout << "at iteration: (" << i << "," << j << ") : " << a << std::endl;
            }

        }
    }
    return subImage;
}

mvtVec C_blockMatching::getMotionVector(C_imgMatrix<double> subImage, int dL, int dC)
{
    //init search
    mvtVec dX;
    dX.dL=-dL;
    dX.dC=-dC;
    double m = INFINITY;
    double tmp;

    //for all the research area [-dL,dL]*[-dC,dC]
    for(int dl=-dL ; dl<=dL ; dl++)
    {
        for(int dc=-dC ; dc<=dC ; dc++)
        {
            //compute a similarity function: interframe difference
            try
            {
                tmp = getBlockInterFrameDifference(subImage, dl, dc); //this method may throw an exception
            }
            catch(char const* a)
            {
                std::cout << "getMotionVector"  << std::endl;
                std::cout << dl << " " << dc << " " << a << std::endl;
                throw a; //forward exception
            }

            if(tmp<m)
            {
                m=tmp;
                dX.dL=-dl;
                dX.dC=-dc;
            }
        }
    }
    return dX;
}




C_imgMatrix<double> C_blockMatching::createNextFrame(int nbBlockL, int nbBlockC, int dL, int dC)
{
    //create the patchwork container
    C_imgMatrix<double> patch(image1.getNbRow(),image1.getNbColumn());

    //divide image into blocks
    std::vector<C_imgMatrix<double> > subDiv = divideImage(nbBlockL, nbBlockC);

    //make sure that the allowed motion is not too large... this may be improved
    if(dL+1>=subDiv.at(0).getNbRow()+1 || dC+1>subDiv.at(0).getNbColumn())
    {
        std::cout << "the method can not handle this case" << std::endl;
    }
    else
    {
        for(size_t i=0 ; i<subDiv.size() ; i++)
        {
            //get motion vector for ith sub-image
            mvtVec toto = getMotionVector(subDiv.at(i), dL, dC);

            //change frame
            toto.dL = -toto.dL;
            toto.dC = -toto.dC;

            //check boundaries
            int lbegin, lend, cbegin, cend;
            lbegin = subDiv.at(i).offsetL+toto.dL;
            lend = subDiv.at(i).offsetL+subDiv.at(i).getNbRow()+toto.dL-1;
            cbegin = subDiv.at(i).offsetC+toto.dC;
            cend = subDiv.at(i).offsetC+subDiv.at(i).getNbColumn()+toto.dC-1;

            int subLbegin = 0;
            int subCbegin = 0;
            int subLend = subDiv.at(i).endL;
            int subCend = subDiv.at(i).endC;


            if(lbegin<0)
            {
                subLbegin = -lbegin;
                lbegin = 0;
            }
            if(cbegin<0)
            {
                subCbegin = -cbegin;
                cbegin = 0;
            }
            if(lend>patch.endL)
            {
                subLend = subLend - (lend-patch.endL);
                lend = patch.endL;
            }
            if(cend>patch.endC)
            {
                subCend = subCend - (cend-patch.endC);
                cend = patch.endC;
            }


            //extract the subset of the block that can be paste into the patchwork
            C_imgMatrix<double> tmpPatch = subDiv.at(i).subset(subLbegin, subLend, subCbegin, subCend);
            try
            {
                //paste it the right place
                patch.subset(tmpPatch, lbegin, lend, cbegin, cend);
            }
            catch(char const* a)
            {
                std::cout << a << std::endl;
            }
        }
    }
    //return the composite image
    return patch;
}
