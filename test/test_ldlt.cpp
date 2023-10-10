#include <iostream>
#include <eigen3/Eigen/Dense>

// A modified Cholesky decomposition is used for symmetric indefinite matrices. 
// The objective is to factor the matrix into the form P^T(D+E)P = A 
// where P is a permutation matrix, D is a diagonal matrix, 
// and E is a positive definite correction matrix. 
// This approach attempts to approximate the matrix A as close as possible 
// to a positive definite matrix.

// bool ModifiedCholeskyRookPivoting(const Eigen::MatrixXd& A, Eigen::MatrixXd& L, Eigen::MatrixXd& D, double tol) {
//     if (A.rows() != A.cols()) {
//         std::cerr << "Matrix is not square!" << std::endl;
//         return false;
//     }

//     int n = A.rows();
//     Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n, n);
//     L = Eigen::MatrixXd::Identity(n, n);
//     D = Eigen::MatrixXd::Zero(n, n);
//     Eigen::MatrixXd AP = A;

//     for (int j = 0; j < n; j++) {
//         // Find the pivot
//         Eigen::ArrayXd colMax = AP.col(j).tail(n-j).array().abs();
//         Eigen::ArrayXd rowMax = AP.row(j).tail(n-j).array().abs();
//         int p = j + colMax.maxCoeff();
//         int q = j + rowMax.maxCoeff();
//         if (AP(p, j) < -tol || AP(p, j) > tol) {
//             q = j;
//         } else if (!(AP(j, q) < -tol || AP(j, q) > tol)) {
//             std::cerr << "Matrix is not factorizable using this method." << std::endl;
//             return false;
//         }

//         // Swap rows and columns
//         AP.row(j).swap(AP.row(p));
//         AP.col(j).swap(AP.col(p));
//         P.row(j).swap(P.row(p));

//         // Factorization
//         L.block(j+1, j, n-j-1, 1) = AP.block(j+1, j, n-j-1, 1) / AP(j, j);
//         D(j, j) = AP(j, j);
//         AP.block(j+1, j+1, n-j-1, n-j-1) -= L.block(j+1, j, n-j-1, 1) * (AP(j, j+1));
//     }

//     L = P.transpose() * L;

//     return true;
// }

bool isPositiveDefiniteMatrix(const Eigen::MatrixXd& A) {
    if (A.rows() != A.cols()) {
        std::cerr << "Matrix is not square!" << std::endl;
        return false;
    }

    int n = A.rows();
    bool bPositive = true;

    for (int i = 1; i<=n; i++){
        if (A.block(0,0,i,i).determinant() <= 0) {
            bPositive = false;
            break;
        }
    }

    return bPositive;
}

int main() {
    Eigen::MatrixXd A(3, 3);
    A << 4, 2, -2,
         2, 10, 4,
         -2, 4, 9;

    std::cout << "Eigen(A) = " << A.eigenvalues()<< std::endl;

    if(isPositiveDefiniteMatrix(A)) {
        std::cout << A << " is positive definite!" << std::endl;
    }
    else {
        std::cout << A << " is NOT positive definite!" << std::endl;
    }


    // double tol = 1e-10;
    // Eigen::MatrixXd L, D;

    // if (ModifiedCholeskyRookPivoting(A, L, D, tol)) {
    //     std::cout << "L:\n" << L << std::endl;
    //     std::cout << "D:\n" << D << std::endl;

    //     Eigen::MatrixXd reconstructedA = L * D * L.transpose();
    //     std::cout << "Reconstructed A:\n" << reconstructedA << std::endl;
    // }

    return 0;
}
