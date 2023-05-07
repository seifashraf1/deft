#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cuda_runtime.h>
#include <cuda.h>

__global__ void matrixMultiplication(int** M, int* Msummary, int n, int alpha, int omega) {
    // Calculate the indices of the current thread
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    
    // Perform matrix multiplication for the current thread's position
    if (row < n && col < n) {
        int result = M[omega][row * n + col];
        
        for (int i = omega - 1; i >= alpha; i--) {
            result *= M[i][row * n + col];
        }
        
        Msummary[row * n + col] = result;
    }
}

void fillMatrices(int*** h_M, int*** d_M, int numMatrices, int n)
{
    srand(time(NULL));
    *h_M = new int*[numMatrices];
    *d_M = new int*[numMatrices];
    
    for (int m = 0; m < numMatrices; m++)
    {
        (*h_M)[m] = new int[n * n];
        cudaMalloc(&((*d_M)[m]), n * n * sizeof(int));

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                (*h_M)[m][i * n + j] = rand() % 2;  // Random value 0 or 1
            }
        }

        cudaMemcpy((*d_M)[m], (*h_M)[m], n * n * sizeof(int), cudaMemcpyHostToDevice);
    }
}

int main() {
    // Assuming M is a 3D array of matrices, where M[numMatrices][n][n]
    int numMatrices = 5; // Number of matrices
    int n = 10; // Size of each matrix
    int omega = numMatrices - 1; // Value of alpha
    int alpha = 0;
    
    // Allocate memory for the input matrices M on the host
    int*** h_M = new int**[numMatrices];
    for (int i = 0; i < numMatrices; i++) {
        h_M[i] = new int*[n];
        for (int j = 0; j < n; j++) {
            h_M[i][j] = new int[n];
        }
    }
    
    // Fill the input matrices with some values
    srand(time(NULL));
    for (int m = 0; m < numMatrices; m++)
    {
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                h_M[m][i][j] = rand() % 2;  // Random value 0 or 1
            }
        }
    }
    
    // Allocate memory for the input and output matrices on the device
    int*** d_M;
    int* d_Msummary;
    cudaMalloc(&d_M, numMatrices * sizeof(int**));
    for (int i = 0; i < numMatrices; i++) {
        cudaMalloc(&(d_M[i]), n * n * sizeof(int));
    }
    cudaMalloc(&d_Msummary, n * n * sizeof(int));
    
    // Copy the input matrices from the host to the device
    for (int i = 0; i < numMatrices; i++) {
        cudaMemcpy(d_M[i], h_M[i], n * n * sizeof(int), cudaMemcpyHostToDevice);
    }
    
    dim3 blockSize(16, 16);
    dim3 gridSize((n + blockSize.x - 1) / blockSize.x, (n + blockSize.y - 1) / blockSize.y);

    // Allocate memory for d_Msummary
    int** d_Msummary;
    cudaMalloc((void**)&d_Msummary, n * sizeof(int*));
    for (int i = 0; i < n; i++)
        cudaMalloc((void**)&(d_Msummary[i]), n * sizeof(int));

    // Call the matrix multiplication CUDA kernel
    matrixMultiplication<<<gridSize, blockSize>>>(d_M, d_Msummary, n, alpha, omega);

    
    // Copy the result matrix from the device to the host
    int* h_Msummary = new int[n * n];
    cudaMemcpy(h_Msummary, d_Msummary, n * n * sizeof(int), cudaMemcpyDeviceToHost);
    
    // Free the allocated memory on the device
    for (int i = 0; i < numMatrices; i++) {
        cudaFree(d_M[i]);
    }
    cudaFree(d_Msummary);
    
    // Use the resulting Msummary matrix
    
    // Free the allocated memory on the host
    for (int i = 0; i < numMatrices; i++) {
        for (int j = 0; j < n; j++) {
            delete[] h_M[i][j];
        }
        delete[] h_M[i];
    }
    delete[] h_M;
    delete[] h_Msummary;

    return 0;
}