
#include <stdio.h>
#define X_LENGTH 2
#define Y_LENGTH 2
const unsigned int xlength = 2;
const unsigned int ylength = 2;

int main()
{

  int matrix[X_LENGTH][Y_LENGTH]={{1,2},{3,4}};

  for(int i = 0;i<X_LENGTH;i++){
    for(int j = 0;j<Y_LENGTH;j++){
      printf("val=%d\n",*(*(matrix+i)+j));
    }
  }
}
