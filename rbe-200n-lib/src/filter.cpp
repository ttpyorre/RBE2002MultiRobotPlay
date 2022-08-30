#include "filter.h"

void AveragingFilter::addDatum(float value)
{
    //add to the circular buffer
    data[filterIndex++] = value;
    if(filterIndex == dataSize) filterIndex = 0;
}

float AveragingFilter::addAndReturnAverage(float value)
{
    addDatum(value);

    float  average = 0;
    for(int i = 0; i < dataSize; i++)
    {
        average += data[i];
    }

    return average / dataSize;
}

float AveragingFilter::addAndReturnMedian(float value)
{
    addDatum(value);

    //we're going to sort the last dataSize data points, but note
    //that we don't sort the original array, as that would mess up our time series  
    //this is a brute force method -- there are better ways 

    //first copy data into a temporary array 
    float tempArray[dataSize];
    for(int i = 0; i < dataSize; i++)
    {
        tempArray[i] = data[i];
    }

    //now brute force sort the temporary array
    for(int i = 0; i < dataSize; i++)
    {
        for(int j = i + 1; j < dataSize; j++)
        {
            if(tempArray[j] < tempArray[i]) std::swap(tempArray[i], tempArray[j]);
        }
    }

    return tempArray[dataSize / 2];
}