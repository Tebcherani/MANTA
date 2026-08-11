#include <lapacke.h>

#include "matrix_utils.h"

int compute_matrix_inverse(double a[][7], double ainv[][7], int n) {
    double matrix[36];
    lapack_int pivots[6];
    lapack_int result;
    int i, j;

    if (n < 1 || n > 6)
        return 1;

    // Convert legacy 1-based matrix to a normal row-major matrix
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            matrix[i * n + j] = a[i + 1][j + 1];
        }
    }

    // Compute LU factorization
    result = LAPACKE_dgetrf(
        LAPACK_ROW_MAJOR,
        n,
        n,
        matrix,
        n,
        pivots
    );

    if (result != 0)
        return 1;

    // Compute inverse from the LU factorization
    result = LAPACKE_dgetri(
        LAPACK_ROW_MAJOR,
        n,
        matrix,
        n,
        pivots
    );

    if (result != 0)
        return 1;

    // Convert back to the simulator's legacy 1-based matrix
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            ainv[i + 1][j + 1] = matrix[i * n + j];
        }
    }

    return 0;
}