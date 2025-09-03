#include "chainspring.hpp"

void CHAIN_1D::build_M_K()
{
    /*Build Mass Matrix*/
    M = m.asDiagonal();
    /*Build stiffness Matrix*/
    K.resize(node, node);
    K.setZero();
    K(0, 0) = k(0);
    K(0, 1) = -k(0);
    for (int i = 1; i < node - 1; i++)
    {
        K(i, i - 1) = -k(i - 1);
        K(i, i) = k(i - 1) + k(i);
        K(i, i + 1) = -k(i);
    }
    K(node - 1, node - 1) = k(node - 2);
    K(node - 1, node - 2) = -k(node - 2);

    // std::cout << M << std::endl;
    // std::cout << K << std::endl;
}

void CHAIN_1D::next_position()
{
    Eigen::MatrixXd B = h * K * (X - x0 + h * V);
    Eigen::VectorXd delta_v = lltA.solve(B);
    V = V - delta_v;
    X = X + V * h;
    std::cout << std::endl
              << V << std::endl;
    std::cout << X << std::endl;
}
