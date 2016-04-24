#include <iostream>
#include <stdio.h>
#include <sstream>

#include <C_imgMatrix.h>
#include <C_motionEstimation.h>
#include <C_blockMatching.h>

#include "C_modelEstimation.h"


//#define BLOCKMATCHING
//#define MOTIONESTIMATION
//#define MODELESTIMATION
#define EX_MATRIX_OPERATIONS //les indices commencent à 0
//#define EX_IMAGE_OPERATIONS


#ifdef EX_MATRIX_OPERATIONS
int main(int argc, char *argv[])
{
    //create a matrix 3x3
    C_matrix<double> A(3,3);
    //set its entries
    A(0,0)=1.0; A(0,1)=1.0; A(0,2)=0.0;
    A(1,0)=0.0; A(1,1)=2.0; A(1,2)=1.0;
    A(2,0)=0.0; A(2,1)=0.0; A(2,2)=0.5;

    //display in the prompt
    A.show();

    //calculate the determinant
    std::cout << "Determinant: " << A.CalcDeterminant() << std::endl;

    //compute the inverse using Gauss-Jordan pivot method (inv2 uses the comatrix method)
    A.inv().show();

    //make sure the calculated matrix is the inverse
    (A*(A.inv())).show();

    //create a second matrix
    C_matrix<double> B(3,3);
    //set all entries to 1.5
    B=1.5;

    //add and show the result
    (A+B).show();
    //save in a file
    (A+B).save("addAB");
    //create a third matrix that contains the matrix product A*B
    C_matrix<double> C=(A*B);
    C.show();

    //create another matrix that contains the elementwise product A.*B
    C_matrix<double> D(A*B);
    D.show();
    return 0;
}
#endif

#ifdef EX_IMAGE_OPERATIONS
int main(int argc, char *argv[])
{
    if(argc!=2)
    {
        std::cout << "Command " << argv[0] << " must be run: " << argv[0] << " image name<string>" << std::endl;
        return -1;
    }
    std::string file1 = argv[1];

    //create an image by loading the file file1
    C_imgMatrix<double> im1(file1);

    //display the image in a pop up window
    im1.display(NO_NORMALIZATION);

    //save the image
    im1.savepng("toto.png");

    //because C_imgMatrix is the daughter of C_matrix, you can use all the methods of C_matrix, including the operators (+,-,*,(),>,...)
    //copy im1 into im2
    C_imgMatrix<double> im2=im1;
    //threshold the im2
    im2=(im2>128.0)*255.0;//NOTE: for multiplying a matrix by a scalar, the scalar must be placed on the right side of the operator *
    //show the thresholded image
    im2.display(NORMALIZE);
    //save im2 as png
    im2.savepng("totoTH.png");

    //get a subset of the image
    C_imgMatrix<double> im3=im1.subset(0,im2.endL/2,0,im2.endC/2);
    im3.display(NORMALIZE);

    //change a subset of the image
    im2.subset(im3,im2.endL-im3.endL,im2.endL,im2.endC-im3.endC,im2.endC);
    im2.display(NORMALIZE);
    im2.savepng("totoHybrid.png");
    return 0;
}
#endif


#ifdef BLOCKMATCHING
int main(int argc, char *argv[])
{
    if(argc!=7)
    {
        std::cout << "Command " << argv[0] << " must be run: " << argv[0] << " frame1<string> frame2<string> block rows <int> block column <int> search perimeter row <int> seach perimeter <column>" << std::endl;
        return -1;
    }
    std::string file1 = argv[1];
    std::string file2 = argv[2];

    //show the file names that will be loaded
    std::cout << "Input files: " << file1 << " " << file2 << std::endl;

    //block matching
    C_blockMatching blckMtch(file1 /**first frame*/, file2 /**second frame*/);

    //try different value of the parameters and observe the results
    C_imgMatrix<double> patch = blckMtch.createNextFrame(atoi(argv[3])/*#row blocks*/,atoi(argv[4])/*#column blocks*/,atoi(argv[5])/*row area param*/,atoi(argv[6])/*column area param*/);
    //C_imgMatrix<double> patch = blckMtch.createNextFrame(15/*#row blocks*/,15/*#column blocks*/,12/*row area param*/,12/*column area param*/);

    //display the estimated second frame
    patch.display(NO_NORMALIZATION);

    //show the difference between predicted frame and actual frame
    C_imgMatrix<double> diff((blckMtch.image2-patch).m_abs());
    diff.display(NO_NORMALIZATION);

    patch.savepng("patch.png");
    return 0;
}
#endif

#ifdef MOTIONESTIMATION
int main(int argc, char *argv[])
{
    if(argc!=4)
    {
        std::cout << "Command " << argv[0] << " must be run: " << argv[0] << " frame1<string> frame2<string> method <0:laplacian, 1:neighbour V8, 2:Horn and Shunck>" << std::endl;
        return -1;
    }
    std::string file1 = argv[1];
    std::string file2 = argv[2];

    //show the file names that will be loaded
    std::cout << "Input files: " << file1 << " " << file2 << std::endl;

    C_motionEstimation mvt(file1 /**first frame*/, file2 /**second frame*/, 0.025 /**time between frames*/);

    //motion estimation using the original image and its Laplacian
    if(atoi(argv[3])==LAP)
    {
        mvt.computeMotionFieldLap();
        if(mvt.saveVectorField("vectXLap","vectYLap")) std::cout << "the results are saved in the files: vectXLap and vectYLap" << std::endl;
    }

    //motion estimation using the 8-neighboring system (try this method on null-divergence )
    if(atoi(argv[3])==V8)
    {
        mvt.computeMotionFieldV8();
        if(mvt.saveVectorField("vectXV8","vectYV8")) std::cout << "the results are saved in the files: vectXV8 and vectYV8" << std::endl;
    }

    //motion estimation using Horn and Schunck's method (try this method using different value of the regularizing parameter, and try different images)
    if(atoi(argv[3])==HNS)
    {
        double alpha;
        int N;
        std::cout << "please enter the regularization weight: ";
        std::cin >> alpha;
        std::cout << std::endl;
        std::cout << "please enter the number of iterations: ";
        std::cin >> N;
        mvt.computeMotionFieldHnS(alpha,N);
        if(mvt.saveVectorField("vectXHnS","vectYHnS")) std::cout << "the results are saved in the files: vectXHnS and vectYHnS" << std::endl;
    }

    //use octave/matlab to plot (quiver) a vector field on an image, for instance:
    //load vectXHnS
    //load vectYHnS
    //I1=imread("the_image1")
    //I2=imread("the_image2")
    //figure(1)
    //imshow(abs(I2-I1))
    //quiver(vectYHnS,vectXHnS)

    return 0;
}
#endif

#ifdef MODELESTIMATION
int main(int argc, char *argv[])
{
    if(argc!=4)
    {
        std::cout << "Command " << argv[0] << " must be run: " << argv[0] << " frame1<string> frame2<string> method <0:laplacian, 1:neighbour V8, 2:Horn and Shunck>" << std::endl;
        return -1;
    }
    std::string file1 = argv[1];
    std::string file2 = argv[2];

    //show the file names that will be loaded
    std::cout << "Input files: " << file1 << " " << file2 << std::endl;


    //motion estimation using the original image and its Laplacian
    if(atoi(argv[3])==LAP)
    {
        C_modelEstimation mvt(file1 /**first frame*/, file2 /**second frame*/, 0.025 /**time between frames*/, LAP /*method for the estimation of the motion data*/);
        //mvt.computeMotionFieldLap();
        if(mvt.saveVectorField("vectXLap","vectYLap")) std::cout << "the motion data are saved in the files: vectXLap and vectYLap" << std::endl;
        //for the purpose of the example, create a Region Of Interest of the size of the image
        ROI R(1.0,1.0);
        for(int l=0 ; l<mvt.vectX.getNbRow() ; l++)
        {
            for(int c=0 ; c<mvt.vectX.getNbColumn() ; c++)
            {
                R.idxL.push_back(l);
                R.idxC.push_back(c);
                R.x.push_back((double)l);
                R.y.push_back((double)c);
            }
        }
        R.N=R.idxL.size();

        //compute the parameter estimation for the ROI
        mvt.computeAfineEstimation(R);
        mvt.saveMaps();

        //
        mvt.image1.display(NO_NORMALIZATION);
        C_imgMatrix<double> aff=mvt.applyAffineModel(50.0*0.10, 0.0, 0.0, 0.0, 50.0*0.10, 0.0);
        aff.display(NO_NORMALIZATION);
        aff.savepng("aff.png");
        mvt.image2=aff;
    }

    //motion estimation using the 8-neighboring system (try this method on null-divergence )
    if(atoi(argv[3])==V8)
    {
        C_modelEstimation mvt(file1 /**first frame*/, file2 /**second frame*/, 0.025 /**time between frames*/, V8 /*method for the estimation of the motion data*/);
        //mvt.computeMotionFieldV8();
        if(mvt.saveVectorField("vectXV8","vectYV8")) std::cout << "the motion data are saved in the files: vectXV8 and vectYV8" << std::endl;
    }

    //motion estimation using Horn and Schunck's method (try this method using different value of the regularizing parameter, and try different images)
    if(atoi(argv[3])==HNS)
    {
        double alpha;
        int N;
        std::cout << "please enter the regularization weight: ";
        std::cin >> alpha;
        std::cout << std::endl;
        std::cout << "please enter the number of iterations: ";
        std::cin >> N;
        C_modelEstimation mvt(file1 /**first frame*/, file2 /**second frame*/, 0.025 /**time between frames*/, V8 /*method for the estimation of the motion data*/,alpha,N);
        //mvt.computeMotionFieldHnS(alpha,N);
        if(mvt.saveVectorField("vectXHnS","vectYHnS")) std::cout << "the motion data are saved in the files: vectXHnS and vectYHnS" << std::endl;
    }

    //use octave/matlab to plot (quiver) a vector field on an image, for instance:
    //load vectXHnS
    //load vectYHnS
    //I1=imread("the_image1")
    //I2=imread("the_image2")
    //figure(1)
    //imshow(abs(I2-I1))
    //quiver(vectYHnS,vectXHnS)

    return 0;
}
#endif



