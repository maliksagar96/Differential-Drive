
#include "movingArray.h"

void init_movingArray(int arraySize, int n) {
	int i;
	for(i = 0;i < arraySize-1;i++){
		movArray[i][n] = 0;
	}
	size[n] = arraySize;
	index[n] = 0;
	total[maxN] = 0;
}

void addElement(float element, int n) {
	total[n] -= movArray[index[n]][n];
	movArray[index[n]][n] = element;
	total[n] += element;
	index[n]++;
	if(index[n] == size[n]) {
		index[n] = 0;
	}
}

float getTotal(int n) {
	return total[n];
}

float getAverage(int n) {
	return total[n] / size[n];
}

// returns f(t - x) th element
float getElement(int x, int n) {
	return movArray[index[n] - x][n];
}
