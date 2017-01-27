
#ifndef movingArray
#define movingArray

#define maxN 2
#define maxArraySize 10

int size[maxN];
float movArray[maxArraySize][maxN];
int index[maxN];
float total[maxN];

void init_movingArray(int arraySize, int n);
void addElement(float element, int n);
float getTotal(int n);
float getAverage(int n);
float getElement(int x, int n);


#endif