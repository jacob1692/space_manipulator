#include "chaser_spacedyn/datasave.h"

DataSave::DataSave()
{
    //- Check -//
    qDebug() << "Start DataSave Init";

    //=== Open File to save simulation data ===//
    //=========================================//
    fout.open("aData/data.txt"); //  <-- Check
    if(!fout){
        qDebug() << "Error! I can't open the file for Output";
        exit(1);
    }

    //=== Initialize Data ===//
    //=======================//
    initData();

    //- Check -//
    qDebug() << "End DataSave Init";
}

DataSave::~DataSave()
{

}

void DataSave::initData()
{
    //== Output basic information ==//

}

void DataSave::saveData(float t)
{
    //======================================//
    //=== Output simulation data to file ===//
    //======================================//
    fout << t << " ";
    fout << m.POS0[0] << " ";
    fout << m.POS0[1] << " ";
    fout << m.POS0[2] << " ";
    fout << Q0_deg[0] << " ";
    fout << Q0_deg[1] << " ";
    fout << Q0_deg[2] << " ";
    fout << POS_e[0] << " ";
    fout << POS_e[1] << " ";
    fout << POS_e[2] << " ";
    fout << Qe_deg[0] << " ";
    fout << Qe_deg[1] << " ";
    fout << Qe_deg[2] << " ";
    fout << q_deg[0] << " ";
    fout << q_deg[1] << " ";
    fout << endl;
}
