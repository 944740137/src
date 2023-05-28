#include <iostream>
using namespace std;

//#define BENCHMARK

#include "GP.h"
#include "CGOptimizer.h"
#include "armadillo_backsub.h"

GP::GP(REAL s2_n, KernelFunction *kernel, MeanFunction *mean)
{
  this->kernel = kernel;
  this->mean = mean;
  this->s2_n = s2_n;
  this->count = 0;
  this->need_to_compute_alpha = false;
  this->need_to_compute_chol = false;
}

void GP::SetKernelFuncParams(const Col<REAL>& param)
{
  this->kernel->SetParams(param);
  
  MatrixMap(this->Kf, this->X, this->X, this->U.row(0));
  MatrixMap(this->Kg, this->X, this->X, this->U.row(1));
  MatrixMap(this->Kg2, this->X, this->X, this->U.row(2));
  K = Kf + Kg + Kg2;
  K += this->s2_n*eye<Mat<REAL> >(X.n_cols, X.n_cols);

  this->need_to_compute_alpha = true;
  this->need_to_compute_chol = true;
}


void GP::SetMeanFuncParams(const Col<REAL>& param)
{
  this->mean->SetParams(param);

  this->gg2meanvals.set_size(this->X.n_cols);
  this->gmeanvals.set_size(this->X.n_cols);
  this->meanvals.set_size(this->X.n_cols);

  for (unsigned int i = 0; i < this->X.n_cols; ++i)
  {
      this->meanvals(i) = this->mean->Eval(this->X.col(i));
      this->gmeanvals(i) = this->mean->gEval(this->X.col(i));
      this->gg2meanvals(i) = this->mean->gg2Eval(this->X.col(i));
  }

  this->need_to_compute_alpha = true;
}

void GP::SetNoise(const REAL &s2_n)
{
  K -= this->s2_n*eye<Mat<REAL> >(X.n_cols, X.n_cols);
  this->s2_n = s2_n;
  K += this->s2_n*eye<Mat<REAL> >(X.n_cols, X.n_cols);

  this->need_to_compute_alpha = true;
  this->need_to_compute_chol = true;
}

void GP::SetTraining(const Mat<REAL>& X, const Row<REAL> &y, const Mat<REAL>& U)
{
  this->y.set_size(y.n_cols);
  //this->U.set_size(U.n_cols);

  MatrixMap(Kf, X, X, U.row(0));
  MatrixMap(Kg, X, X, U.row(1));
  MatrixMap(Kg2, X, X, U.row(2));
  K = Kf + Kg + Kg2;
  K += this->s2_n * eye<Mat<REAL> >(X.n_cols, X.n_cols);
  //cout << "Ktr:\n" << K << endl;

  this->X = X;
  this->y = y;
  this->U = U;

  this->gg2meanvals.set_size(this->X.n_cols);
  this->gmeanvals.set_size(this->X.n_cols);
  this->meanvals.set_size(X.n_cols);
  for (unsigned int i = 0; i < X.n_cols; ++i)
  {
      this->meanvals(i) = this->mean->Eval(X.col(i));
      this->gmeanvals(i) = this->mean->gEval(this->X.col(i));
      this->gg2meanvals(i) = this->mean->gg2Eval(this->X.col(i));
  }
  //cout<< "fmean:\n" << this->meanvals << endl;
  //cout << "gmean:\n" << this->gmeanvals << endl;

  this->need_to_compute_alpha = true;
  this->need_to_compute_chol = true;
}

void GP::AddTraining(const Mat<REAL>& X, const Row<REAL> &y, const Mat<REAL>& utr)
{
  int orig_n_cols, orig_n_rows;
  int new_n_cols, new_n_rows;
  count++;
  
  if (count <= 50)
  {
      if (!this->y.is_empty())
      {
          // cout << "y:\n" << this->y << endl;
           // Extend y
          Row<REAL> tempy;
          tempy = this->y;
          orig_n_cols = this->y.n_cols;
          new_n_cols = orig_n_cols + y.n_cols;
          this->y.set_size(new_n_cols);
          this->y.cols(0, orig_n_cols - 1) = tempy;
          this->y.cols(orig_n_cols, new_n_cols - 1) = y;
          // cout << "y:\n" << this->y << endl;
      }
      else
      {
          this->y = y;
      }

      if (!this->meanvals.is_empty())
      {
          // cout << "meanf:\n" << this->meanvals << endl;
          // Extend & Compute meanvals
          orig_n_cols = this->meanvals.n_cols;
          new_n_cols = orig_n_cols + X.n_cols;
          Row<REAL> tempmean;
          tempmean = this->meanvals;
          this->meanvals.set_size(new_n_cols);
          this->meanvals.cols(0, orig_n_cols - 1) = tempmean;
          for (unsigned int i = 0; i < X.n_cols; ++i)
              this->meanvals(orig_n_cols + i) = this->mean->Eval(X.col(i));
          // cout << "meanf:\n" << this->meanvals << endl;
      }
      else
      {
          this->meanvals.set_size(X.n_cols);
          for (unsigned int i = 0; i < X.n_cols; ++i)
              this->meanvals(i) = this->mean->Eval(X.col(i));
      }

      if (!this->gmeanvals.is_empty())
      {
          //cout << "meang:\n" << this->gmeanvals << endl;
          // Extend & Compute gmeanvals
          orig_n_cols = this->gmeanvals.n_cols;
          new_n_cols = orig_n_cols + X.n_cols;
          Row<REAL> tempgmean;
          tempgmean = this->gmeanvals;
          this->gmeanvals.set_size(new_n_cols);
          this->gmeanvals.cols(0, orig_n_cols - 1) = tempgmean;
          for (unsigned int i = 0; i < X.n_cols; ++i)
              this->gmeanvals(orig_n_cols + i) = this->mean->gEval(X.col(i));
          //cout << "meang:\n" << this->gmeanvals << endl;
      }
      else
      {
          this->gmeanvals.set_size(X.n_cols);
          for (unsigned int i = 0; i < X.n_cols; ++i)
              this->gmeanvals(i) = this->mean->gEval(X.col(i));
      }

      if (!this->gg2meanvals.is_empty())
      {
          //cout << "meang:\n" << this->gmeanvals << endl;
          // Extend & Compute gmeanvals
          orig_n_cols = this->gg2meanvals.n_cols;
          new_n_cols = orig_n_cols + X.n_cols;
          Row<REAL> tempgg2mean;
          tempgg2mean = this->gg2meanvals;
          this->gg2meanvals.set_size(new_n_cols);
          this->gg2meanvals.cols(0, orig_n_cols - 1) = tempgg2mean;
          for (unsigned int i = 0; i < X.n_cols; ++i)
              this->gg2meanvals(orig_n_cols + i) = this->mean->gg2Eval(X.col(i));
          //cout << "meang:\n" << this->gmeanvals << endl;
      }
      else
      {
          this->gg2meanvals.set_size(X.n_cols);
          for (unsigned int i = 0; i < X.n_cols; ++i)
              this->gg2meanvals(i) = this->mean->gg2Eval(X.col(i));
      }

      if (!this->X.is_empty())
      {
          //cout << "X:\n" << this->X << endl;
          // Extend X
          Mat<REAL> tempX = this->X;
          orig_n_cols = this->X.n_cols;
          orig_n_rows = this->X.n_rows;
          new_n_cols = orig_n_cols + X.n_cols;
          new_n_rows = orig_n_rows;
          this->X.set_size(new_n_rows, new_n_cols);
          this->X.submat(0, 0, orig_n_rows - 1, orig_n_cols - 1) = tempX;
          this->X.submat(0, orig_n_cols, new_n_rows - 1, new_n_cols - 1) = X;
          //cout << "X:\n" << this->X << endl;
      }
      else
      {
          this->X = X;
      }

      if (!this->U.is_empty())
      {
          //cout << "U:\n" << this->U << endl;
          //Extend U
          Mat<REAL> tempU = this->U;
          orig_n_cols = this->U.n_cols;
          orig_n_rows = this->U.n_rows;
          new_n_cols = orig_n_cols + utr.n_cols;
          new_n_rows = orig_n_rows;
          this->U.set_size(new_n_rows, new_n_cols);
          this->U.submat(0, 0, orig_n_rows - 1, orig_n_cols - 1) = tempU;
          this->U.submat(0, orig_n_cols, new_n_rows - 1, new_n_cols - 1) = utr;
          //cout << "U:\n" << this->U << endl;
      }
      else
      {
          this->U = utr;
      }
  }
  else
  {
      int index = count % 50;
      if (index)
      {
          this->y.col(index - 1) = y;
          this->X.col(index - 1) = X;
          this->U.col(index - 1) = utr;
      }
      else
      {
          this->y.col(50 - 1) = y;
          this->X.col(50 - 1) = X;
          this->U.col(50 - 1) = utr;
      }
  }

 // cout << "K:\n" << this->K << endl;
  MatrixMap(this->Kf, this->X, this->X, this->U.row(0));
  MatrixMap(this->Kg, this->X, this->X, this->U.row(1));
  MatrixMap(this->Kg2, this->X, this->X, this->U.row(2));
  this->K = this->Kf + this->Kg + this->Kg2;
  this->K += this->s2_n*eye<Mat<REAL> >(this->X.n_cols, this->X.n_cols);
  //cout << "K:\n" << this->K << endl;

  this->need_to_compute_alpha = true;
  this->need_to_compute_chol = true;
}

void GP::ComputeChol()
{
  if(this->need_to_compute_chol) {
#ifdef BENCHMARK
    wall_clock timer;
    timer.tic();
#endif
    //cout << "K: \n" << K << endl;
    L = chol(K);
    this->need_to_compute_w = true;
    this->need_to_compute_chol = false;
#ifdef BENCHMARK
    printf("%f seconds to compute cholesky\n", timer.toc());
#endif
  }
}

void GP::ComputeAlpha()
{
  if(this->need_to_compute_alpha) {
    // Ensure that L is up to date
    ComputeChol();
#ifdef BENCHMARK
    wall_clock timer;
    timer.tic();
#endif
    this->alpha.set_size(y.n_cols);
    cholbacksub(alpha, this->L, trans(this->y - this->U.row(0) % this->meanvals - this->U.row(1) % this->gmeanvals - this->U.row(2) % this->gg2meanvals));
    //cout << "alpha: \n" << alpha << endl;
    this->need_to_compute_alpha = false;
#ifdef BENCHMARK
    printf("%f seconds to compute alpha\n", timer.toc());
#endif
  }
}

void GP::ComputeW()
{
  if(this->need_to_compute_w) {
    ComputeChol();
    this->W.set_size(this->L.n_rows, this->L.n_cols);
    cholbacksub(W, this->L, eye<Mat<REAL> >(K.n_rows, K.n_cols));
    this->need_to_compute_w = false;
  }
}

REAL GP::ComputeLikelihood()
{
  ComputeChol();
  ComputeAlpha();

  this->loglikelihood = -0.5*dot( (this->y-this->meanvals), this->alpha);
  this->loglikelihood += -accu(log(L.diag()));
  this->loglikelihood += -0.5*this->y.n_elem*log(2*Math<REAL>::pi());

  return this->loglikelihood;
}

void GP::GradLikelihoodMeanParams(Col<REAL> &grad)
{
  ComputeAlpha();
  this->mean->Grad(grad, X.col(0));

  Col<REAL> tmp;
  Mat<REAL> grad_tmp(grad.n_rows, X.n_cols);
  for(unsigned int i=0; i < X.n_cols; ++i) {
    this->mean->Grad(tmp, X.col(i));
    grad_tmp.col(i) = -tmp;
  }

  grad = grad_tmp*this->alpha;
}

void GP::HessianLikelihoodMeanParams(Mat<REAL> &hessian)
{
  ComputeAlpha();
  ComputeW();

  Col<REAL> grad;
  this->mean->Grad(grad, X.col(0));

  // Get Gradient and hessian of mean function at each training point X
  // with current parameters \theta
  Mat<REAL> grad_all(grad.n_elem, X.n_cols);
  std::vector<Col<REAL> > grad_tmp(X.n_cols);
  std::vector<Mat<REAL> > hessian_tmp(X.n_cols);
  for(unsigned int i=0; i < X.n_cols; ++i) {
    grad_tmp[i].set_size(grad.n_rows);
    hessian_tmp[i].set_size(grad.n_rows, grad.n_rows);

    this->mean->Grad(grad_tmp[i], X.col(i));
    grad_all.col(i) = grad_tmp[i];
    this->mean->Hessian(hessian_tmp[i], X.col(i));
  }

  // grad_tmp[k] is the gradient of m(X_i, theta)
  // hessian_tmp[i] is the hessian of m(X_i, theta)

  hessian.set_size(grad.n_rows, grad.n_rows);
  hessian.fill(0);
  for(unsigned int i=0; i < grad.n_rows; ++i) {
    for(unsigned int j=0; j < grad.n_rows; ++j) {

      for(unsigned int k=0; k < alpha.n_elem; ++k) {
        hessian(i,j) += hessian_tmp[k](i,j)*alpha(k);
      }

      hessian(i,j) -= dot(grad_all.row(i), this->W*trans(grad_all.row(j)));
    }
  }
}

void GP::GradLikelihoodKernelParams(Col<REAL> &grad)
{
  ComputeAlpha();
  ComputeW();

  // Do this to get the number of params
  this->kernel->Grad(grad, X.col(0), X.col(1));

  std::vector<Mat<REAL> > partialK(grad.n_elem);

  for(unsigned int k=0; k < grad.n_elem; ++k) {
    partialK[k].set_size(X.n_cols, X.n_cols);
  }

  for(unsigned int i=0; i < X.n_cols; ++i) {
    for(unsigned int j=0; j < X.n_cols; ++j) {
      this->kernel->Grad(grad, X.col(i), X.col(j));
      for(unsigned int k=0; k < grad.n_elem; ++k) {
        partialK[k](i,j) = grad(k);
      }
    }
  }

  for(unsigned int k=0; k < grad.n_elem; ++k) {
    Mat<REAL> tmp = trans(alpha)*partialK[k]*alpha;
    grad(k) = 0.5*(tmp(0,0)) - 0.5*trace(W*partialK[k]);
  }
}

void GP::GradLikelihoodNoise(Col<REAL> &grad)
{
  ComputeW();
  grad.set_size(1);

  Mat<REAL> partialK = 2*this->s2_n*eye<Mat<REAL> >(this->X.n_cols, this->X.n_cols);
  Mat<REAL> tmp = trans(alpha)*partialK*alpha;
  grad(0) = 0.5*(tmp(0,0)) - 0.5*trace(W*partialK);
}

void GP::Predict(const Mat<REAL> &Xs, REAL &mu, REAL & gmu, REAL& gg2mu)
{
#ifdef BENCHMARK
  wall_clock timer;
  timer.tic();
#endif

 /* mu.set_size(Xs.n_cols);
  gmu.set_size(Xs.n_cols);*/

  REAL mu1;
  REAL gmu1;
  REAL gmu2;

  for(unsigned int i=0; i < Xs.n_cols; ++i) {
      this->Predict(Xs.col(i), this->U, mu1, gmu1, gmu2);
    if(isinf(mu1)) {
      printf("INF?!?: %f %f\n", Xs(0,i), Xs(1,i));
    }
    mu = mu1;
    gmu = gmu1;
    gg2mu = gmu2;
  }

#ifdef BENCHMARK
  printf("%f seconds to predict\n", timer.toc());
#endif
}

void GP::Predict(const Mat<REAL> &Xs, Row<REAL> &mu, Row<REAL> &var, Row<REAL>& gmu, Row<REAL>& gg2mu)
{

#ifdef BENCHMARK
  wall_clock timer;
  timer.tic();
#endif

  mu.set_size(Xs.n_cols);
  var.set_size(Xs.n_cols);
  gmu.set_size(Xs.n_cols);
  gg2mu.set_size(Xs.n_cols);

  REAL mu1, var1, gmu1, gmu2;

  for(unsigned int i=0; i < Xs.n_cols; ++i) {
      this->Predict(Xs.col(i), this->U, mu1, var1, gmu1, gmu2);
    mu(i) = mu1;
    var(i) = var1;
    gmu(i) = gmu1;
    gg2mu(i) = gmu2;
  }

#ifdef BENCHMARK
  printf("%f seconds to predict (with var)\n", timer.toc());
#endif
}

void GP::Predict(const Col<REAL> &Xs, const Mat<REAL> &utr, REAL &mu, REAL &gmu, REAL& gg2mu)
{
  ComputeChol();
  ComputeAlpha();

  Col<REAL> kstar(this->X.n_cols);
  for (unsigned int i = 0; i < this->X.n_cols; ++i)
      kstar(i) = this->kernel->Eval(Xs, this->X.col(i));

  //cout << "U = \n" << utr << endl;
  Col<REAL> utr1 = trans(utr.row(0));
  Col<REAL> utr2 = trans(utr.row(1));
  Col<REAL> utr3 = trans(utr.row(2));
  //cout << "utr1 = \n" << utr1 << endl;
  //cout << "utr2 = \n" << utr2 << endl;

  mu = dot(utr1 % kstar, alpha) + this->mean->Eval(Xs);
  gmu = dot(utr2 % kstar, alpha) + this->mean->gEval(Xs);
  gg2mu = dot(utr3 % kstar, alpha) + this->mean->gg2Eval(Xs);
 // cout << "ww = \n" << this->mean->gEval(Xs) << endl;
  //cout << "qq = \n" << dot(utr2 % kstar, alpha) << endl;
  
}

void GP::Predict(const Col<REAL> &Xs, const Mat<REAL> &utr, REAL &mu, REAL &var, REAL &gmu, REAL& gg2mu)
{
  ComputeChol();
  ComputeAlpha();

  Col<REAL> kstar(X.n_cols);
  for (unsigned int i = 0; i < X.n_cols; ++i)
      kstar(i) = this->kernel->Eval(Xs, X.col(i));

  mu = dot(kstar,alpha) + this->mean->Eval(Xs);
  gmu = dot(kstar, alpha) + this->mean->Eval(Xs);

  // trans(L) is LOWER TRIANGULAR
  Col<REAL> v(kstar.n_elem);
  solve_tri(v, trans(L), kstar, false);
  REAL kxsxs = this->kernel->Eval(Xs, Xs);
  var = kxsxs - dot(v, v) + this->s2_n;
}

void GP::PredictGradient(const Col<REAL> &Xs, Col<REAL> &grad)
{
  grad.set_size(Xs.n_elem);

  ComputeChol();
  ComputeAlpha();

  Mat<REAL> dkstar(grad.n_rows, X.n_cols);
  for(unsigned int i=0; i < X.n_cols; ++i) {
    Col<REAL> tmp;
    this->kernel->GradX(tmp, Xs, X.col(i));
    dkstar.col(i) = tmp;
  }

  this->mean->GradX(grad, Xs);

  for(unsigned int i=0; i < grad.n_elem; ++i) {
    grad(i) += dot(dkstar.row(i), alpha);
  }
}

void GP::PredictGradient(const Col<REAL> &Xs, Col<REAL> &grad, Col<REAL> &vargrad)
{
  grad.set_size(Xs.n_elem);
  vargrad.set_size(Xs.n_elem);

  ComputeChol();
  ComputeAlpha();


  Col<REAL> kstar(X.n_cols);
  Mat<REAL> dkstar(grad.n_rows, X.n_cols);
  for(unsigned int i=0; i < X.n_cols; ++i) {
    Col<REAL> tmp;
    this->kernel->GradX(tmp, Xs, X.col(i));
    dkstar.col(i) = tmp;

    kstar(i) = this->kernel->Eval(Xs, X.col(i));
  }

  this->mean->GradX(grad, Xs);

  for(unsigned int i=0; i < grad.n_elem; ++i) {
    grad(i) += dot(dkstar.row(i), alpha);
  }

  Col<REAL> v(kstar.n_elem);
  Col<REAL> tmp(kstar.n_elem);
  Mat<REAL> dv(kstar.n_elem, grad.n_elem);
  solve_tri(v, trans(L), kstar, false);
  
  //printf("About to solve for dv\n");
  solve_tri(tmp, trans(L), trans(dkstar.row(0)), false);
  dv.col(0) = tmp;
  solve_tri(tmp, trans(L), trans(dkstar.row(1)), false);
  dv.col(1) = tmp;
  //dv.print("dv: ");

  Col<REAL> dkxsxs;
  this->kernel->GradX(dkxsxs, Xs, Xs);

  vargrad = dkxsxs - 2*trans(dv)*v;
  //v.print("v: ");
}

void GP::MatrixMap(Mat<REAL> &matrix, const Mat<REAL> &a, const Mat<REAL> &b, const Row<REAL>& U)
{
  Mat<REAL> Umatrix;
  Umatrix.set_size(U.n_cols, U.n_cols);
  for (unsigned int i = 0; i < U.n_cols; ++i) {
      for (unsigned int j = 0; j < U.n_cols; ++j) {
          Umatrix(i, j) = U(0, i) * U(0, j);
      }
  }

  //cout << "U:\n" << Umatrix << endl;

  Mat<REAL> Kmatrix;
  Kmatrix.set_size(a.n_cols, b.n_cols);
  for(unsigned int i=0; i < a.n_cols; ++i) {
    for(unsigned int j=0; j < b.n_cols; ++j) {
      REAL val = this->kernel->Eval(a.col(i), b.col(j));
      Kmatrix(i, j) = val;
    }
  }

  //cout << "K:\n" << Kmatrix << endl;

  matrix.set_size(a.n_cols, b.n_cols);
  matrix = Umatrix % Kmatrix;

 // cout << "Kg:\n" << matrix << endl;

    /*matrix.set_size(a.n_cols, b.n_cols);
    for (unsigned int i = 0; i < a.n_cols; ++i) {
        for (unsigned int j = 0; j < b.n_cols; ++j) {
            REAL val = this->kernel->Eval(a.col(i), b.col(j));
            matrix(i, j) = val;
        }
    }*/
}

void GP::OptimizeNoiseParam(REAL &noise_param, int max_iterations)
{
  CGOptimizer opt(reinterpret_cast<void*>(this));

  Col<REAL> noise_params(1);
  noise_params(0) = noise_param;

  opt.Initialize(noise_params, &f_eval_noise, &df_eval_noise, &fdf_eval_noise, 0.5, 0.1);
  opt.Optimize(noise_params, max_iterations);
}

void GP::OptimizeMeanParam(Col<REAL> &mean_param, int max_iterations)
{
  CGOptimizer opt(reinterpret_cast<void*>(this));

  opt.Initialize(mean_param, &f_eval_mean, &df_eval_mean, &fdf_eval_mean, 1.5, 0.01);
  opt.Optimize(mean_param, max_iterations);
}

void GP::OptimizeKernelParam(Col<REAL> &kernel_param, int max_iterations)
{
  CGOptimizer opt(reinterpret_cast<void*>(this));

  opt.Initialize(kernel_param, &f_eval_kernel, &df_eval_kernel, &fdf_eval_kernel, 20, 0.1);
  opt.Optimize(kernel_param, max_iterations);
}

double f_eval_mean(const gsl_vector *x, void *param)
{
  GP *gp_obj = reinterpret_cast<GP*>(param);

  Col<REAL> mean_param(gp_obj->GetMeanFunction()->GetParamDim());
  for(unsigned int i=0; i < mean_param.n_elem; ++i) {
    mean_param(i) = gsl_vector_get(x, i);
  }

  gp_obj->SetMeanFuncParams(mean_param);

  double ret = -gp_obj->ComputeLikelihood();

  return ret;
}

void df_eval_mean(const gsl_vector *x, void *param, gsl_vector *g)
{
  GP *gp_obj = reinterpret_cast<GP*>(param);

  Col<REAL> mean_param(gp_obj->GetMeanFunction()->GetParamDim());
  for(unsigned int i=0; i < mean_param.n_elem; ++i) {
    mean_param(i) = gsl_vector_get(x, i);
  }
  gp_obj->SetMeanFuncParams(mean_param);

  Col<REAL> grad;
  gp_obj->GradLikelihoodMeanParams(grad);

  for(unsigned int i=0; i < grad.n_elem; ++i) {
    gsl_vector_set(g, i, -grad(i));
  }
}

void fdf_eval_mean(const gsl_vector *x, void *param, double *f, gsl_vector *g)
{
  *f = f_eval_mean(x, param);
  df_eval_mean(x, param, g);
}

double f_eval_kernel(const gsl_vector *x, void *param)
{
  GP *gp_obj = reinterpret_cast<GP*>(param);

  Col<REAL> kernel_param(gp_obj->GetKernelFunction()->GetParamDim());
  for(unsigned int i=0; i < kernel_param.n_elem; ++i) {
    kernel_param(i) = gsl_vector_get(x, i);
    if(kernel_param(i) < 1e-6) {
      return 1e6;
    }
  }

  gp_obj->SetKernelFuncParams(kernel_param);

  double ret = -gp_obj->ComputeLikelihood();

  return ret;
}

void df_eval_kernel(const gsl_vector *x, void *param, gsl_vector *g)
{
  GP *gp_obj = reinterpret_cast<GP*>(param);

  Col<REAL> kernel_param(gp_obj->GetKernelFunction()->GetParamDim());
  for(unsigned int i=0; i < kernel_param.n_elem; ++i) {
    kernel_param(i) = gsl_vector_get(x, i);
  }
  gp_obj->SetKernelFuncParams(kernel_param);

  Col<REAL> grad;
  gp_obj->GradLikelihoodKernelParams(grad);

  for(unsigned int i=0; i < grad.n_elem; ++i) {
    gsl_vector_set(g, i, -grad(i));
  }
}

void fdf_eval_kernel(const gsl_vector *x, void *param, double *f, gsl_vector *g)
{
  *f = f_eval_kernel(x, param);
  df_eval_kernel(x, param, g);
}

double f_eval_noise(const gsl_vector *x, void *param)
{
  GP *gp_obj = reinterpret_cast<GP*>(param);

  REAL noise_param;
  noise_param = gsl_vector_get(x, 0);
  if(noise_param < 1e-6) {
    return 1e6;
  }

  gp_obj->SetNoise(noise_param);

  double ret = -gp_obj->ComputeLikelihood();

  return ret;
}

void df_eval_noise(const gsl_vector *x, void *param, gsl_vector *g)
{
  GP *gp_obj = reinterpret_cast<GP*>(param);

  REAL noise_param;
  noise_param = gsl_vector_get(x, 0);

  gp_obj->SetNoise(noise_param);

  Col<REAL> grad;
  gp_obj->GradLikelihoodNoise(grad);

  for(unsigned int i=0; i < grad.n_elem; ++i) {
    gsl_vector_set(g, i, -grad(i));
  }
}

void fdf_eval_noise(const gsl_vector *x, void *param, double *f, gsl_vector *g)
{
  *f = f_eval_noise(x, param);
  df_eval_noise(x, param, g);
}
