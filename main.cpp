#include <Eigen/Dense>
#include "chainspring.hpp"
#include <igl/opengl/glfw/Viewer.h>

int main()
{
    igl::opengl::glfw::Viewer viewer;
    Eigen::VectorXd v(4);
    v << 0, 0, 0, 2;
    Eigen::VectorXd x(4);
    x << -1, 0, 1, 2;
    Eigen::VectorXd m(4);
    m << 1, 2, 3, 4;
    Eigen::VectorXd k(3);
    k << 3, 4, 5;
    Eigen::MatrixXd P(4, 3);
    P.col(0) = x;
    P.col(1).setZero();
    P.col(2).setZero();
    CHAIN_1D test(4, v, x, m, k, 0.1);
    double dt = 0.1;
    viewer.data().add_points(P, Eigen::RowVector3d(0, 1, 0));
    viewer.data().add_edges(P.row(0), P.row(1), Eigen::RowVector3d(1, 0, 0));
    viewer.data().add_edges(P.row(1), P.row(2), Eigen::RowVector3d(1, 0, 0));
    viewer.data().add_edges(P.row(2), P.row(3), Eigen::RowVector3d(1, 0, 0));
    viewer.core().camera_zoom = 0.3;
    viewer.data().point_size = 8;
    auto last_update = std::chrono::high_resolution_clock::now();
    viewer.core().is_animating = true;
    bool new_p = true;
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer)
    {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - last_update;

        if (elapsed.count() >= dt)
        {
            viewer.data().clear_edges();
            viewer.data().clear_points();
            x = test.Getposition();
            P.col(0) = x;
            P.col(1).setZero();
            P.col(2).setZero();
            viewer.data().add_points(P, Eigen::RowVector3d(0, 1, 0));
            viewer.data().add_edges(P.row(0), P.row(1), Eigen::RowVector3d(1, 0, 0));
            viewer.data().add_edges(P.row(1), P.row(2), Eigen::RowVector3d(1, 0, 0));
            viewer.data().add_edges(P.row(2), P.row(3), Eigen::RowVector3d(1, 0, 0));
            last_update = now;
            new_p = true;
        }

        return false;
    };

    viewer.callback_post_draw = [&](igl::opengl::glfw::Viewer &viewer)
    {
        if (new_p)
        {
            test.next_position();
            new_p = false;
        }
        return false;
    };
    viewer.launch();

    // for (int i = 0; i < 10; i++)
    // {
    //     std::cout << i * 0.1 << std::endl;
    //     test.next_position();
    // }
}