//
// Created by Noureddine Gueddach on 11/12/2022.
//

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <igl/doublearea.h>
#include <igl/massmatrix.h>
#include <igl/barycenter.h>
#include <igl/cotmatrix.h>
#include <igl/write_triangle_mesh.h>

class MeshProcessor {
public:

    //Using default params
    MeshProcessor() {}

    void smoothMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, Eigen::MatrixXd& V_impLap) {
        V_impLap = V;
        Eigen::SparseMatrix<double> L;
        igl::cotmatrix(V, F, L);

        float delta = 0.00001; //Controls the smoothing amount

        // Compute Mass Matrix
        Eigen::SparseMatrix<double> M;
        igl::massmatrix(V_impLap, F, igl::MASSMATRIX_TYPE_BARYCENTRIC, M);
        // Solve (M-delta*L) U = M*U
        const auto & S = (M - delta * L);
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver(S);
        assert(solver.info() == Eigen::Success);
        V_impLap = solver.solve(M * V_impLap).eval();
        // Compute centroid and subtract (also important for numerics)
        Eigen::VectorXd dblA;
        igl::doublearea(V, F, dblA);
        double area = 0.5 * dblA.sum();
        Eigen::MatrixXd BC;
        igl::barycenter(V_impLap, F, BC);
        Eigen::RowVector3d centroid(0, 0, 0);
        for(int i = 0; i < BC.rows(); i++) {
            centroid += 0.5 * dblA(i) / area * BC.row(i);
        }
        V_impLap.rowwise() -= centroid;
    }

    void saveMesh(const std::string& path, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) const {
        igl::write_triangle_mesh(path, V, F);
    }

private:

};