#include <iostream>
#include <math.h>

using namespace std;




const int M_SIZE = 5;

struct mat{
  int row;
  int col;
  float data[SIZE][SIZE];
};


const int V_SIZE = 3;

struct vect{
  int len;
  float data[V_SIZE];
};








void clearArray(float Array[SIZE][SIZE]) {
  for (int r = 0; r < SIZE; r += 1) {
    for (int c = 0; c < SIZE; c += 1) {
      Array[r][c] = 0;
    }
  }
}


void clearMatrix(mat &M) {
  for (int r = 0; r < SIZE; r += 1) {
    for (int c = 0; c < SIZE; c += 1) {
      M.data[r][c] = 0;
    }
  }
}


void copyArrayToMatrix(mat &d_matrix, float s_array[SIZE][SIZE]) {

  for (int r = 0; r < SIZE; r += 1) {
    for (int c = 0; c < SIZE; c += 1) {
      d_matrix.data[r][c] = s_array[r][c];
    }
  }
}


void copyMatrix(mat &d_matrix, mat &s_matrix) {

  for (int r = 0; r < SIZE; r += 1) {
    for (int c = 0; c < SIZE; c += 1) {
      d_matrix.data[r][c] = s_matrix.data[r][c];
    }
  }
}




void printMatrix(mat &M){
  int row = M.row;
  int col = M.col;

  Serial.println(String(row) + " by " + String(col) + " Matrix");
    for (int r = 0; r < row; r += 1) {
      for (int c = 0; c < col; c += 1) {
        Serial.print(String(M.data[r][c]) + "\t");
      }
      Serial.println();
    }
    Serial.println();
}





void addMatrix(mat &result, mat &matrix1, matrix &matrix2) {
  if( (matrix1.row == matrix2.row) && (matrix1.col == matrix2.col) ){
    float buffer[SIZE][SIZE];
    clearArray(buffer);

    int row = matrix1.row;
    int col = matrix1.col;

    for (int r = 0; r < row; r += 1) {
      for (int c = 0; c < col; c += 1) {
        buffer[r][c] = matrix1.data[r][c] + matrix2.data[r][c];
      }
    }

    clearMatrix(result);
    copyMatrix(buffer, result);
  }

  else {
    cout << "invalid operation" << endl;
  }
}



void subMatrix(matrix &matrix1, matrix &matrix2, matrix &result) {
  if( (matrix1.row == matrix2.row) && (matrix1.col == matrix2.col) ){
    float buffer[SIZE][SIZE];
    clearArray(buffer);

    int row = matrix1.row;
    int col = matrix1.col;

    for (int r = 0; r < row; r += 1) {
      for (int c = 0; c < col; c += 1) {
        buffer[r][c] = matrix1.data[r][c] - matrix2.data[r][c];
      }
    }

    clearMatrix(result);
    copyMatrix(buffer, result);
  }

  else {
    cout << "invalid operation" << endl;
  }
}




void transposeMatrix(matrix &matrix1, matrix &result) {
  
  float buffer[SIZE][SIZE];
  clearArray(buffer);

  int row = matrix1.row;
  int col = matrix1.col;

  for (int r = 0; r < col; r += 1) {
      for (int c = 0; c < row; c += 1) {
        buffer[r][c] = matrix1.data[c][r];
      }
    }

    result.row = col;
    result.col = row;
  clearMatrix(result);
  copyMatrix(buffer, result);
}




void scaleMatrix(float scalar, matrix &matrix1, matrix &result) {

  float buffer[SIZE][SIZE];
  clearArray(buffer);

  int row = matrix1.row;
  int col = matrix1.col;

  for (int r = 0; r < row; r += 1) {
    for (int c = 0; c < col; c += 1) {
      buffer[r][c] = matrix1.data[r][c] * scalar;
    }
  }

  clearMatrix(result);
  copyMatrix(buffer, result);

}


void scaleMatrixDiv(float scalar, matrix &matrix1, matrix &result) {

  float buffer[SIZE][SIZE];
  clearArray(buffer);

  int row = matrix1.row;
  int col = matrix1.col;

  for (int r = 0; r < row; r += 1) {
    for (int c = 0; c < col; c += 1) {
      buffer[r][c] = matrix1.data[r][c] / scalar;
    }
  }

  clearMatrix(result);
  copyMatrix(buffer, result);

}






void dotMatrix(matrix &matrix1, matrix &matrix2, matrix &result) {
  if(matrix1.col == matrix2.row){

    int new_row = matrix1.row;
    int new_col = matrix2.col;

    float buffer[SIZE][SIZE];
    clearArray(buffer);

    float val = 0;

    matrix T_matrix2;
    transposeMatrix(matrix2, T_matrix2);
    int loop = T_matrix2.col;

    for (int r=0; r<new_row; r+=1) {
        for (int c=0; c<new_col; c+=1) {
            for (int count = 0; count < loop; count += 1) {
              val += matrix1.data[r][count] * T_matrix2.data[c][count];
            }
            buffer[r][c] = val;
            val = 0;
        }
      }

      result.row = new_row;
      result.col = new_col;
    clearMatrix(result);
    copyMatrix(buffer, result);
  }

  else {
    cout << "invalid operation" << endl;
  }
}






void minor_element(int pos_r, int pos_c, matrix &major_matrix, matrix &minor_matrix) {
  if(major_matrix.row == major_matrix.col){

    int minor_r = 0, minor_c = 0;

      int row = major_matrix.row;
      int col = major_matrix.col;

      minor_matrix.row = row-1;
      minor_matrix.col = col-1;

      float buffer[SIZE][SIZE];
      clearArray(buffer);

      if(row>=2){
          for(int r=0; r<row; r+=1){

              for(int c=0; c<col; c+=1){
                  if((r==pos_r) || (c==pos_c)) {
                      continue;
                  }
                  else {
                      buffer[minor_r][minor_c] = major_matrix.data[r][c];
                      minor_c+=1;
                      if(minor_c>=minor_matrix.col) minor_c = 0;
                  }
              }

             if(r==pos_r) {
                      continue;
              }
              else {
                  minor_r+=1;
                  if(minor_r>=minor_matrix.row) minor_r = 0;
              } 
          }
      }

      clearMatrix(minor_matrix);
      copyMatrix(buffer, minor_matrix);
  }

  else {
    cout << "invalid operation" << endl;
  }
      
}










float det1(matrix &matrix1) {
  if(matrix1.row == matrix1.col){
    float d=0;

      d = matrix1.data[0][0];
      return d;
  }
      
}



float det(matrix &matrix1) {
  if(matrix1.row == matrix1.col){

    if (matrix1.row ==1){
      det1(matrix1);
    }
  
        else {
            float d=0;
            matrix minor_matrix;
            for (int c=0; c<matrix1.col; c+=1){
                minor_element(0,c, matrix1, minor_matrix);
                if(c==0 || (c%2)==0) d+=(matrix1.data[0][c]*det(minor_matrix));
                else d-=(matrix1.data[0][c]*det(minor_matrix));
            }
            return d;
        }

     
  }


    else {
        cout << "invalid matrix determinant operation" << endl;
        return 0;
    }   

}



void minor_matrix(matrix &major_matrix, matrix &minor_matrix){
  if(major_matrix.row == major_matrix.col){

    int row = major_matrix.row;
    int col = major_matrix.col;

    float buffer[SIZE][SIZE];
    clearArray(buffer);

    matrix minor;
    clearMatrix(minor);
    
    for(int r=0; r<row; r+=1){
      for(int c=0; c<col; c+=1){
        minor_element(r,c, major_matrix, minor);  
        buffer[r][c] = det(minor);
      }
    }

    minor_matrix.row = major_matrix.row;
    minor_matrix.col = major_matrix.col;

    clearMatrix(minor_matrix);
    copyMatrix(buffer, minor_matrix);
    
  }

      
    else {
        cout << "invalid matrix operation" << endl;
    }  
}



void cofactor_matrix(matrix &minor_matrix, matrix &co_matrix){

  if(minor_matrix.row == minor_matrix.col){

    int row = minor_matrix.row;
    int col = minor_matrix.col;

    float buffer[SIZE][SIZE];
    clearArray(buffer);
    
    for(int r=0; r<row; r+=1){
      for(int c=0; c<col; c+=1){
        if((r+c)==0 || ((r+c)%2)==0) buffer[r][c] = minor_matrix.data[r][c];
                else buffer[r][c] = -1*minor_matrix.data[r][c];
      }
    }

    co_matrix.row = minor_matrix.row;
    co_matrix.col = minor_matrix.col;

    clearMatrix(co_matrix);
    copyMatrix(buffer, co_matrix);
    
  }
   
    else {
        cout << "invalid matrix operation" << endl;
    }  

}


void adjoint_matrix(matrix &co_matrix, matrix &ad_matrix){

  if(co_matrix.row == co_matrix.col){
    transposeMatrix(co_matrix, ad_matrix);
  }
      
    else {
        cout << "invalid matrix operation" << endl;
    }  

}



void inverseMatrix(matrix &matrix1, matrix &inverse){

  if(matrix1.row == matrix1.col){
    float d = det(matrix1);

    matrix buffer;
    minor_matrix(matrix1, buffer);
    cofactor_matrix(buffer, buffer);
    adjoint_matrix(buffer, buffer);

    inverse.row = matrix1.row;
    inverse.col = matrix1.col;

    scaleMatrixDiv(d, buffer, inverse);
  }
      
    else {
        cout << "invalid matrix operation" << endl;
    }  

}















//int main(){
//  matrix A;
//  A.row = 2; A.col = 2;
//  float buffer1[SIZE][SIZE] = {
//    {3, -4},
//    {2, -5},
//  };
//  clearMatrix(A);
//  copyMatrix(buffer1, A);
//
//  matrix B;
//  B.row = 3; B.col = 3;
//  float buffer2[SIZE][SIZE] = {
//    {1,3,2},
//    {4,1,3},
//    {2,5,2},
//  };
//  clearMatrix(B);
//  copyMatrix(buffer2, B);
//
//  matrix C;
//  C.row = 4; C.col = 4;
//  float buffer3[SIZE][SIZE] = {
//    {3,2,0,1},
//    {4,0,1,2},
//    {3,0,2,1},
//    {9,2,3,1},
//  };
//  clearMatrix(C);
//  copyMatrix(buffer3, C);
//
//  matrix P;
//  // matrix Q;
//  
//  // // printMatrix(B);
//  // minor_element(0,0, A, P);
//  // printMatrix(A);
//  // Q.row = A.row;
//  // Q.col = A.col;
//  // clearMatrix(Q);
//  // Q.data[0][0] = det(P);
//  // printMatrix(Q);
//  // cout << det(P) << endl;
//
//  printMatrix(C);
//  cout << det(C) << endl;
//  inverseMatrix(C, P);
//  printMatrix(P);
//
//  return 0;
//}
