#ifndef DATASAVE_H
#define DATASAVE_H

//=== C++ ===//
#include <iostream>
#include <fstream>
using namespace std;
//=== Others ===//
#include "chaser_spacedyn/dataext.h"    // Global and define

class DataSave
{
public:
    //===== Constructor and Destructor =====//
    DataSave();
    ~DataSave();
    //======================================//
    void initData();
    void sampleData();
    void saveData(float t);
    void printData();

protected:

private:
    ofstream fout;
    ifstream fin;
    int i,j;
};

#endif // DATASAVE_H
