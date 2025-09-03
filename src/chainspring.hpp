#include <Eigen/Dense>
#include <iostream>
#include <Eigen/src/Cholesky/LLT.h>

class CHAIN_1D
{
private:
    int node;
    // boundary
    Eigen::VectorXd v0;
    Eigen::VectorXd x0;
    // Recent status
    Eigen::VectorXd X;
    Eigen::VectorXd V;
    // basic point mass and stiffness
    Eigen::VectorXd m;
    Eigen::VectorXd k;
    // The mass matrix and stiffness matrix
    Eigen::MatrixXd M;
    Eigen::MatrixXd K;
    // Pre-cal the Matrix
    Eigen::LLT<Eigen::MatrixXd> lltA;
    // foot step
    double h;
    void build_M_K();

public:
    CHAIN_1D(int n, Eigen::VectorXd v, Eigen::VectorXd x, Eigen::VectorXd m, Eigen::VectorXd k, double h)
        : node(n), v0(v), x0(x), m(m), k(k), h(h)
    {
        build_M_K();
        X = x0;
        V = v0;
        Eigen::MatrixXd A = M + h * h * K;
        lltA.compute(A);
    }
    void next_position();

    Eigen::VectorXd Getposition()
    {
        return X;
    }
};
