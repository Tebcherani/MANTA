#include <lapacke.h>

#include "matrix_utils.h"

int compute_matrix_inverse(double input_matrix[][7], double inverse_matrix[][7], int matrix_size) {
    double reformatted_matrix[36];
    lapack_int pivots[6];
    lapack_int result;
    int i, j;

    if (matrix_size < 1 || matrix_size > 6) return 1;

    // Convert legacy 1-based matrix to a normal row-major matrix
    for (i = 0; i < matrix_size; i++) {
        for (j = 0; j < matrix_size; j++) reformatted_matrix[i * matrix_size + j] = input_matrix[i + 1][j + 1];
    }

    // Compute LU factorization
    result = LAPACKE_dgetrf(LAPACK_ROW_MAJOR, matrix_size, matrix_size, reformatted_matrix, matrix_size, pivots);
    if (result != 0) return 0;

    // Compute inverse from the LU factorization
    result = LAPACKE_dgetri(LAPACK_ROW_MAJOR, matrix_size, reformatted_matrix, matrix_size, pivots);
    if (result != 0) return 0;

    // Convert back to the simulator's legacy 1-based matrix
    for (i = 0; i < matrix_size; i++) {
        for (j = 0; j < matrix_size; j++) inverse_matrix[i + 1][j + 1] = reformatted_matrix[i * matrix_size + j];
    }

    return 1;
}