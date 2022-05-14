#include <iostream>
#include <cmath>
#include <vector>
using namespace std;

vector<float> Weight(const float, const float);//隶属度函数
float Min(const float, const float, const float);//求三个数中最小的数
float WeightChange(const double , const double , const double );//根据车辆当前的状态时时计算w1

float WeightChange(const double dist, const double heading, const double obs=100)
{
    static const float keyDist = 15.0;
    static const float keyHeading = 60.0;
    static const float keyObs = 10.0;

    vector<float> weightDist;
    vector<float> weightHeading;
    vector<float> weightObs;

    weightDist = Weight(keyDist,dist);//a,b
    weightHeading = Weight(keyHeading,heading);
    weightObs = Weight(keyObs,obs);

    float a[8][4];


    a[0][0] = weightDist[0];
    a[1][0] = weightDist[1];
    a[2][0] = weightDist[0];
    a[3][0] = weightDist[1];
    a[4][0] = weightDist[0];
    a[5][0] = weightDist[1];
    a[6][0] = weightDist[0];
    a[7][0] = weightDist[1];

    a[0][1] = weightHeading[1];
    a[1][1] = weightHeading[1];
    a[2][1] = weightHeading[1];
    a[3][1] = weightHeading[1];
    a[4][1] = weightHeading[0];
    a[5][1] = weightHeading[0];
    a[6][1] = weightHeading[0];
    a[7][1] = weightHeading[0];

    a[0][2] = weightObs[0];
    a[1][2] = weightObs[0];
    a[2][2] = weightObs[1];
    a[3][2] = weightObs[1];
    a[4][2] = weightObs[0];
    a[5][2] = weightObs[0];
    a[6][2] = weightObs[1];
    a[7][2] = weightObs[1];

    float w1=80;
    float w2=20;
    float w3=2;
    a[0][3] = 1/w1;
    a[1][3] = 1/w1;
    a[2][3] = 1/w2;
    a[3][3] = 1/w3;
    a[4][3] = w3;
    a[5][3] = w2;
    a[6][3] = w1;
    a[7][3] = w1;

    float miuUp = 0.0;
    float miuDown = 0.0;

    for (int i=0; i<8; i++)
    {
        miuUp += Min(a[i][0],a[i][1],a[i][2])*a[i][3];
        miuDown  +=  Min(a[i][0],a[i][1],a[i][2]);
    }


    float miu;
    miu = miuUp/(miuDown+0.0001);
    return miu;

}

float Min(const float x, const float y, const float z)
{
    float tmp = 0.0;
    tmp = min(x,y);
    tmp = min(tmp,z);
    return tmp;
    
}

vector<float> Weight(const float key, const float x)
{
    vector<float> output;
    float p =4.0;
    float a,b;
    b = 0*(x<=0) + ( pow(2,3) * pow(x/(2*key), p) )* (x>0 & x<key) + (1 - 8*pow((2*key-x)/(2*key ),p) ) * (x>key & x<2*key) + 1*(x>=2*key);
    a = 1- b;
    output.push_back(a);
    output.push_back(b);
    return output;
}

/*int main()
{
    float miu;
    miu = WeightChange(10.0,150.0);
    cout<<miu;
    return 0;
}*/

