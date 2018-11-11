#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <numeric>
#include <list>
#include <vector>
#include <iostream>
#include "Util/log.H"

using namespace Eigen;
using namespace std;

MatrixXd fisher_l_d ( const MatrixXd &X, int comp, vector<int> n_samples)
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // FLD = fisher_l_d ( X, comp, indx )
    //
    // Compute Fisher Linear Discriminant
    // USE:  X data set (each image data is in column vector)
    //       comp nr. of discriminants to be used
    //       n_samples vector storing the nr. of images in each class
    //       FLD vectors where to project data for the best discrimination
    // Author: Markus Weber
    //         Marc'Aurelio Ranzato
    //         Anelia Angelova
    //         Ported from Matlab to eigen3 by D.Cline MBARI 2016
    //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int D = X.rows(); // number of dimensions
    int N = X.cols();  // number of sample points from features
    LINFO("Computing FLD rows %d columns %d", D, N);

    std::vector<int>::iterator data = n_samples.begin();
    int tot_n_samp = std::accumulate(n_samples.begin(), n_samples.end(), 0);
    int lc = n_samples.size();   // nr. of (true) classes
    MatrixXd G;
    VectorXd Ng;//(lc, 1);
    int index2 = 0;

    if (comp < lc) {   // there is no need to split classes
        LINFO("No need to split classes");
        int g = lc;
        Ng.resize(g, 1);
        Ng.setOnes(g, 1);
        G = MatrixXd::Zero(tot_n_samp,g);

        for (int jj=0; jj < lc; jj++) {
            for (int kk=index2; kk < index2 + n_samples[jj]; kk++)
                G(kk, jj) = 1;
            index2 = index2 + n_samples[jj];
            Ng(jj) = n_samples[jj];
        }
    }
    else    // final dimension is greater than lc-1 => split classes in a sufficient nr. of clusters
    {
        LINFO("Splitting classes");
        int num_split = floor(comp / lc) + 1; // nr. of splittings per class
        int g = lc * num_split;
        Ng.resize(g, 1);
        Ng.setOnes(g, 1);
        G = MatrixXd::Zero(tot_n_samp,g);

        int jj;
        for (jj=0; jj < lc; jj++) {
            int nr_s = round(n_samples[jj] / num_split);

            int kk;
            for (kk = 0; kk < num_split - 1; kk++) {

                for (int ll = index2 + 1; ll < (index2 + nr_s); ll++)
                    G(ll, jj * num_split + kk) = 1;

                Ng(jj*num_split + kk) = nr_s;
                index2 = index2 + nr_s;
            }

            // assigning the last data to the last cluster
            nr_s = n_samples[jj] - nr_s * (num_split - 1);

            kk = num_split;
            for (int ll=index2; ll < index2 + nr_s; ll++) {
                G(ll, (jj-1) * num_split + kk) = 1;
            }

            Ng((jj-1)*num_split + kk) = nr_s;
            index2 = index2 + nr_s;
        }

    }

    VectorXd t = N * Ng.array().inverse();//T = diag(sqrt(N ./ Ng));
    t = t.array().sqrt();
    MatrixXd T = t.asDiagonal();

    // Collect Group means
    LINFO("Computing JacobianSVD Thin");
    JacobiSVD<MatrixXd> svd(X.adjoint(), ComputeThinU | ComputeThinV);

    int R = min(D,N);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    VectorXd l = svd.singularValues();
    U = U.leftCols(R);
    V = V.leftCols(R);
    l = l.segment(0,R);

    // Make diagonal out of the singular values
    VectorXd il = l.array().pow(-1); // il = l.^-1;
    MatrixXd iL = il.asDiagonal();
    MatrixXd S = sqrt(N) * V * iL; // S = sqrt(N) * V * iL;
    MatrixXd Xrs = (X.adjoint() * S).adjoint(); // (X' * S)';
    MatrixXd M = (G.adjoint() * G).inverse() * G.adjoint() * Xrs.adjoint();// M = inv(G' * G) * G' * Xrs';

    LINFO("Computing JacobianSVD Full");
    JacobiSVD<MatrixXd> svd2(T.inverse()*M, ComputeFullU | ComputeFullV);
    MatrixXd VV = svd2.matrixV();
    MatrixXd FLD = S * VV;
    //std::cout << "Here is the FLD matrix :\n" << FLD << std::endl;
    FLD = FLD.leftCols(comp);
    return FLD;
}