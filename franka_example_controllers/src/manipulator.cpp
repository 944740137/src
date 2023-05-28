#include <iostream>
#include <fstream>
#include "GP.h"

using namespace std;

int main(int argc, char** argv)
{
    Col<REAL> kernel_param = "5.0 3.0";
    SqExpKernel kernel(kernel_param);
    ConstantMean mean("0,4,1");
    GP gp(0.01, &kernel, &mean);

    SqExpKernel kernel2(kernel_param);
    ConstantMean mean2("0,2,3");
    GP gp2(0.01, &kernel2, &mean2);

    int Nstep = 2000;
    Mat<REAL> Xtr;
    Row<REAL> Ytr1;
    Row<REAL> Ytr2;
    Mat<REAL> Utr;
    Col<REAL> X(4);
    X(0) = 1; X(1) = 1; X(2) = 0.5; X(3) = 0.5;
    REAL dt = 0.01;
    REAL T = 0;
    REAL hatf1, hatf2, hatg11, hatg12, hatg21, hatg22;

    Col<REAL> r(2);
    Col<REAL> rho(2);
    Col<REAL> hatF(2);
    Mat<REAL> hatG;
    hatG.set_size(2, 2);
    Col<REAL> u(2);
    Col<REAL> obstacleBF(2);

    Mat<REAL> H;
    H.set_size(2, 2);
    Mat<REAL> C;
    C.set_size(2, 2);
    Col<REAL> G(2);

    Mat<REAL> allXtr(4, Nstep+1); allXtr(0, 0) = X(0); allXtr(1, 0) = X(1); allXtr(2, 0) = X(2); allXtr(3, 0) = X(3);
    Mat<REAL> allT(1, Nstep+1); allT(0, 0) = T;
    Mat<REAL> alltracking(2, Nstep+1); alltracking(0, 0) = sin(T); alltracking(1, 0) = cos(T);
    Row<REAL> allhatf(1, Nstep);
    Row<REAL> allhatg(1, Nstep);

    for (int nstep = 0; nstep < Nstep; ++nstep)
    {
        if (Xtr.is_empty())
        {
            hatf1 = 0;
            hatf2 = 0;
            hatg11 = 4;
            hatg12 = 1;
            hatg21 = 2;
            hatg22 = 3;
            Xtr.set_size(4, 1);
            Ytr1.set_size(1);
            Ytr2.set_size(1);
            Utr.set_size(3, 1);
        }
        else
        {
            //cout << "Xtr:\n" << Xtr << endl;
            //cout << "Ytr:\n" << Ytr << endl;

            gp.AddTraining(Xtr, Ytr1, Utr);
            gp.Predict(X, hatf1, hatg11, hatg12);

            gp2.AddTraining(Xtr, Ytr2, Utr);
            gp2.Predict(X, hatf2, hatg21, hatg22);
            /*if (!(nstep % 5)) {
                gp.AddTraining(Xtr, Ytr, Utr);
                gp.Predict(X, hatf, hatg);
            }*/
        }

        //cout << "X: \n" << X << endl;
        cout << "第" << nstep << "次" << endl;
        /*cout << "hatf1: \n" << hatf1 << endl;
        cout << "hatf2: \n" << hatf2 << endl;
        cout << "hatg11: \n" << hatg11 << endl;
        cout << "hatg12: \n" << hatg12 << endl;
        cout << "hatg21: \n" << hatg21 << endl;
        cout << "hatg22: \n" << hatg22 << endl;
        cout << endl;*/

        REAL e1 = X(0) - sin(T);
        REAL e2 = X(1) - sin(T);
        REAL e3 = X(2) - cos(T);
        REAL e4 = X(3) - cos(T);
        /*cout << "e1: \n" << e1 << endl;
        cout << "e1: \n" << e2 << endl;
        cout << "e1: \n" << e3 << endl;
        cout << "e1: \n" << e4 << endl;*/
        
        r(0) = e1 + e3; r(1) = e2 + e4;
        rho(0) = e3 + sin(T); rho(1) = e4 + sin(T);

        Col<REAL> nu = -r - rho;
        
        hatF(0) = hatf1; hatF(1) = hatf2;
        hatG(0, 0) = hatg11; hatG(0, 1) = hatg12;
        hatG(1, 0) = hatg21; hatG(1, 1) = hatg22;

        obstacleBF(0) = r(0) / (25 - r(0) * r(0));
        obstacleBF(1) = r(1) / (25 - r(1) * r(1));

        REAL G11 = 9.8 * (8.1 * sin(X(0)) + 1.13 * sin(X(0) + X(1)));
        REAL G21 = 9.8 * (1.13 * sin(X(0) + X(1)));
        G(0) = G11; G(1) = G21;
        /*cout << "G1: \n" << G(0) << endl;
        cout << "G2: \n" << G(1) << endl;*/

        u = G + inv(hatG) * (-hatF + nu) - obstacleBF;
        //cout << "u: \n" << u << endl;

        REAL H11 = 9.77 + 2.02 * cos(X(1));
        REAL H12 = 1.26 + 1.01 * cos(X(1));
        REAL H21 = 1.26 + 1.01 * cos(X(1));
        REAL H22 = 1.12;
        H(0, 0) = H11; H(0, 1) = H12; H(1, 0) = H21; H(1, 1) = H22;
        Col<REAL> gu(2);
        gu = inv(H) * u;

        REAL C11 = -1.01 * sin(X(1)) * X(3);
        REAL C12 = -1.01 * sin(X(1)) * (X(2) + X(3));
        REAL C21 = 1.01 * sin(X(1)) * X(2);
        REAL C22 = 0;
        C(0, 0) = C11; C(0, 1) = C12; C(1, 0) = C21; C(1, 1) = C22;
        Col<REAL> dq(2);
        dq(0) = X(2); dq(1) = X(3);

        Col<REAL> f(2);
        f = -inv(H) * (C * dq + G);

        Col<REAL> dotx34(2);
        REAL dotx1 = X(2);
        REAL dotx2 = X(3);
        dotx34 = f + gu;
        /*cout << "f: \n" << f << endl;
        cout << "g: \n" << gu << endl;*/

        Xtr(0, 0) = X(0); Xtr(1, 0) = X(1); Xtr(2, 0) = X(2); Xtr(3, 0) = X(3);
        Ytr1 = dotx34(0);
        Ytr2 = dotx34(1);
        Utr(0, 0) = 1; Utr(1, 0) = u(0) - G(0); Utr(2, 0) = u(1) - G(1);
        /*cout << "dx3: \n" << dotx34(0) << endl;
        cout << "dx4: \n" << dotx34(1) << endl;
        cout << "Utr: \n" << Utr << endl;*/

        X(0) = X(0) + X(2) * dt;
        X(1) = X(1) + X(3) * dt;
        X(2) = X(2) + dotx34(0) * dt;
        X(3) = X(3) + dotx34(1) * dt;
        T = T + dt;
        /*cout << "X1: \n" << X(0) << endl;
        cout << "X2: \n" << X(1) << endl;
        cout << "X3: \n" << X(2) << endl;
        cout << "X4: \n" << X(3) << endl;*/

        allXtr(0, nstep + 1) = X(0); allXtr(1, nstep + 1) = X(1); allXtr(2, nstep + 1) = X(2); allXtr(3, nstep + 1) = X(3);
        allT(0, nstep + 1) = T;
        alltracking(0, nstep + 1) = sin(T); alltracking(1, nstep + 1) = cos(T);
        allhatf(0, nstep) = hatf1;
        allhatg(0, nstep) = hatg11;
    }

    cout << "Xtr:" << gp.GetTrainingData() << "\n" << "Ytr:" << gp.GetTrainingYData() << "\n" << "Utr:" << gp.GetTrainingUData() << endl;

    /*Row<REAL> ww = allhatf.cols(0, Nstep-2);
    Row<REAL> dd = allhatg.cols(0, Nstep-2);
    Row<REAL> y = gp.GetTrainingYData();
    Mat<REAL> uu = gp.GetTrainingUData();
    Row<REAL> error = y - (ww + dd % uu.row(1));
    cout << "error:\n" << norm(error, 2) << endl;*/

    ofstream dataFile;
    dataFile.open("dataFile.txt", ofstream::app);
    dataFile << "Xtr:" << gp.GetTrainingData() << "\n" << "Ytr:" << gp.GetTrainingYData() << "\n" << "Utr:" << gp.GetTrainingUData() << "\n" << "allT:" << allT << "\n" << "allXtr:" << allXtr << "\n" << "alltracking:" << alltracking << endl;
    dataFile.close();

    //Mat<REAL> X(2, 100);
    //Row<REAL> y;
    //Mat<REAL> u(2, 100);
    //y.set_size(X.n_cols);

    //for (int i = 0; i < X.n_cols; ++i) {
    //    X(0, i) = i;
    //    X(1, i) = i;
    //    y(i) = sin(i * 0.1) + (1 + i * 0.1) * cos(i * 0.1);
    //    u(0, i) = 1;
    //    u(1, i) = 1 + i * 0.1;
    //    //y(i) = 10 + sin(i* Math<REAL>::pi()/3);
    //    //y(i) = 10 + sin(i / (2 * Math<REAL>::pi() * X.n_cols));
    //}

    //Mat<REAL> W(1, 5);
    //Row<REAL> Q;
    //Mat<REAL> utr(2, 5);
    //Q.set_size(W.n_cols);
    //for (int j = 0; j < W.n_cols; ++j) {
    //    W(0, j) = j;
    //    Q(j) = sin(j * 0.1) + (1 + j * 0.1) * cos(j * 0.1);
    //    utr(0, j) = 1;
    //    utr(1, j) = 1 + j * 0.1;
    //}

    //Mat<REAL> Xs = X;
    //Row<REAL> mu, var, gmu;
    //REAL mu1, var1;

    //cout << "Setting training data\n";
    ////gp.SetTraining(X, y, u);
    //gp.AddTraining(X, y, u);
    ////gp.AddTraining(W, Q, utr);
    //cout << "Making predictions\n";
    //gp.Predict(Xs, u, mu, gmu);
    //// gp.Predict(Xs.col(0), mu1, var1);

    //Col<REAL> grad;
    //gp.GradLikelihoodKernelParams(grad);
    //grad.print("Likelihood Gradient (kernel)");

    ////cout << "X:\n" << X << endl;
    ////cout << "y:\n" << y << endl;
    ////cout << "u:\n" << u << endl;
    //cout << "Xtr:\n" << gp.GetTrainingData() << endl;
    //cout << "Ytr:\n" << gp.GetTrainingYData() << endl;
    //cout << "Utr:\n" << gp.GetTrainingUData() << endl;
    ////cout << "W:\n" << W << endl;
    ////cout << "Q:\n" << Q << endl;
    //cout << "Xs:\n" << Xs << endl;
    //cout << "mu:\n" << mu << endl;
    //cout << "gmu:\n" << gmu << endl;
    //cout << "mua:\n" << mu + gmu % u.row(1) << endl;

    //Row<REAL> error = y - (mu + gmu % u.row(1));
    //cout << "error:\n" << norm(error, 2) << endl;

    ////cout << "Log-likelihood: " << gp.ComputeLikelihood() << endl;

    //ofstream dataFile;
    //dataFile.open("dataFile.txt", ofstream::app);
    //dataFile << "X:" << gp.GetTrainingData() << "\n" << "Y:" << gp.GetTrainingYData() << "\n" << "mu:" << mu + gmu % u.row(1) << "\n" << "Utr:" << gp.GetTrainingUData() << endl;
    //dataFile.close();

   // mat A(2, 2), B(2, 2);
   // for (int i = 0; i < 4; i++)
   // {
   //     A(i) = i + 1;//i������Ϊ����˳�򣬾���ĵ�i��Ԫ��
   //     B(i) = i + 5;
   // }
   // Col<REAL> SS(4);
   // Col<REAL> qq(4);
   // for (int i = 0; i < 4; i++)
   // {
   //     SS(i) = i + 2;//i������Ϊ����˳�򣬾���ĵ�i��Ԫ��
   //     qq(i) = i + 3;
   // }
   // cout << A << endl;
   // cout << A.row(0) << endl;
   // cout << A.row(1) << endl;
   // cout << A.col(0) - A.col(1) << endl;
   // cout << norm(A.col(0) - A.col(1), 2) << endl;
   // cout << "A:\n" << A << endl;
   // cout << "B:\n" << B << endl;
   //// cout << "A+B:\n" << A + B << endl;
   //// cout << "A-B:\n" << A - B << endl;
   // //cout << "A*B:\n" << A * B << endl;
   //// cout << "AdotB:\n" << dot(A, B) << endl;
   // cout << "SS:\n" << trans(SS) << endl;
   // cout << "qq:\n" << qq << endl;
   //// //cout << "SSdotB:\n" << dot(SS, B) << endl;
   // cout << "A%B:\n" << A % B << endl;
   //// cout << "A/B:\n" << A / B << endl;

    return 0;
}
