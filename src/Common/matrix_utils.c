#include <math.h>

#include "matrix_utils.h"

int matinv(double a[][7], double ainv[][7], int n, double adum[][7]) {
    int i, j, k;
    int pivot_row;
    int row_swaps = 0;
    int km, kp;
    double pivot;
    double temp;
    double value;
    double max_value;
    double diagonal;
    const double eps = 1.0e-36;

    // Handle 1x1 matrix
    if (n == 1) {
        if (fabs(a[1][1]) <= eps) return 1;
        ainv[1][1] = 1.0 / a[1][1];
        return 0;
    }

    // Copy input matrix into working matrix
    for (i = 1; i <= n; i++) {
        for (j = 1; j <= n; j++) adum[i][j] = a[i][j];
    }

    // Initialize inverse as identity matrix
    for (i = 1; i <= n; i++) {
        for (j = 1; j <= n; j++) ainv[i][j] = (i == j) ? 1.0 : 0.0;
    }

    pivot_row = 1;
    max_value = fabs(adum[1][1]);

    // Find first pivot
    for (i = 2; i <= n; i++) {
        if (fabs(adum[i][1]) > max_value) {
            pivot_row = i;
            max_value = fabs(adum[i][1]);
        }
    }

    // Swap first row if needed
    if (pivot_row != 1) {
        row_swaps++;

        for (j = 1; j <= n; j++) {
            temp = ainv[1][j];
            ainv[1][j] = ainv[pivot_row][j];
            ainv[pivot_row][j] = temp;

            temp = adum[1][j];
            adum[1][j] = adum[pivot_row][j];
            adum[pivot_row][j] = temp;
        }
    }

    pivot = adum[1][1];
    if (fabs(pivot) <= eps) return 1;

    // Normalize first row
    for (j = 2; j <= n; j++) adum[1][j] /= pivot;
    for (j = 1; j <= n; j++) ainv[1][j] /= pivot;

    // Elimination
    for (k = 2; k <= n; k++) {
        km = k - 1;
        pivot_row = k;
        max_value = -1.0;

        for (i = k; i <= n; i++) {
            value = adum[i][k];

            for (j = 1; j <= km; j++) value -= adum[i][j] * adum[j][k];
            adum[i][k] = value;

            if (fabs(value) > max_value) {
                max_value = fabs(value);
                pivot_row = i;
            }
        }

        if (pivot_row != k) {
            row_swaps++;

            for (j = 1; j <= n; j++) {
                temp = ainv[k][j];
                ainv[k][j] = ainv[pivot_row][j];
                ainv[pivot_row][j] = temp;

                temp = adum[k][j];
                adum[k][j] = adum[pivot_row][j];
                adum[pivot_row][j] = temp;
            }
        }

        diagonal = adum[k][k];

        if (fabs(diagonal) <= eps) return 1;

        if (k != n) {
            kp = k + 1;

            for (j = kp; j <= n; j++) {
                value = adum[k][j];
                for (i = 1; i <= km; i++) value -= adum[k][i] * adum[i][j];
                adum[k][j] = value / diagonal;
            }
        }

        for (j = 1; j <= n; j++) {
            value = ainv[k][j];
            for (i = 1; i <= km; i++) value -= adum[k][i] * ainv[i][j];
            ainv[k][j] = value / diagonal;
        }
    }

    // Back substitution
    for (k = 2; k <= n; k++) {
        kp = n - k + 2;
        i = kp - 1;

        for (j = 1; j <= n; j++) {
            value = ainv[i][j];
            for (pivot_row = kp; pivot_row <= n; pivot_row++) value -= adum[i][pivot_row] * ainv[pivot_row][j];
            ainv[i][j] = value;
        }
    }

    return 0;
}