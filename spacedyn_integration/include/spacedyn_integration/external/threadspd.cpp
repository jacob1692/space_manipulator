#include "threadspd.h"
#include <math.h>
#include <cmath>

ThreadSpd::ThreadSpd(QObject *parent) :
    QThread(parent)
{
    //- Check -//
    qDebug() << "Start ThreadSpd Init";

    //== Set FLAG (ThreadSpd stopped) ==//
    //==================================//
    stopped = false;

    //=== Call the model parameters ===//
    //=================================//
    model("aModel/test_model.txt", m);

    //=== Initialize ===//
    //==================//
    initSpd();

    //- Check -//
    qDebug() << "End ThreadSpd Init";
}

ThreadSpd::~ThreadSpd()
{
    delete[] Gravity;
    delete[] A0;
    delete[] ORI_e;
    delete[] POS_e;
    delete[] des_q;
    delete[] des_qd;
    delete[] Q0_rad;
    delete[] Q0_deg;
    delete[] Qe_rad;
    delete[] Qe_deg;
    delete[] q_deg;
    delete[] q_vel;
    flag.F_SPD = false;
}

void ThreadSpd::stopThreadSpd()
{
    mutexSpd.lock();
    stopped = true;
    mutexSpd.unlock();
}


void ThreadSpd::initSpd()
{
    //=== Initialize (Global) ===//
    //===========================//
    model_init(m);

    //s—ñ‚Ì‰Šú‰»
    matrix_Z(6,1,Gravity);                //d—Í‰Á‘¬“xƒxƒNƒgƒ
    // ƒx[ƒXŠp“xŽæ“¾
    matrix_Z(3,3,A0);
    matrix_Z(3,3,ORI_e); 
    matrix_Z(e_num,3,POS_e);
    matrix_Z(2,1,des_q);
    matrix_Z(2,1,des_qd);
    matrix_Z(3,1,Q0_deg);
    matrix_Z(3,1,Q0_rad);
    matrix_Z(3,1,Qe_rad);
    matrix_Z(3,1,Qe_deg);
    matrix_Z(2,1,q_deg);
    matrix_Z(2,1,q_vel);


//    Gravity[2] = 1;
    ////////// ƒƒCƒ“ƒ‹[ƒv“à‚ÅŽg‚¤•Ï”‚ÌéŒ¾ /////////////


    ////////// ‰Šú‰» /////////////////
    // ‰ŠúŠÖßŠp“x‚Ì’è‹`
    m.q[1] = deg2pi(90);
    m.q[2] = deg2pi(-90);

    // –Ú•WŠÖßŠp[rad]
    des_q[0] = deg2pi(-90); //! desired joint position j1
    des_q[1] = deg2pi(90); //! desired joint position j2
    // –Ú•WŠÖßŠp‘¬“x[rad/s]
    des_qd[0] = 0; //! desired joint velocity
    des_qd[1] = 0; //! desired joint velocity

    //    matrix_trans(3,3,m.A0,A0);
    //    dc2rpy(A0,Q0_rad);

    //=== Initialize(SpaceDyn) ===//
    //============================//
    //- Set Timer
    time = 0;
    extime = 0;// ‚±‚±‚Å‰Šú‰»‚µ‚Æ‚©‚È‚¢‚Æmainwidget‚Åtime‚Ì‰Šú’l‚ª‚¨‚©‚µ‚­‚È‚éD65535.00‚Ý‚½‚¢‚È

    d_time = STEPTIME;    f_time = FINISHTIME;

    //- Set Counter
    data_counter = 0;

    signal_init();


}


void ThreadSpd::run()
{
    et.start();
    //- Check thread start
    qDebug() << "Run ThreadSpd";

    //=== ThreadSpd Main loop ===//
    //===========================//
    forever{

        //== CheckFlag(stopped) ==//
        mutexSpd.lock(); // ƒƒbƒN¨ƒƒbƒN’†‚ÍƒXƒŒƒbƒh‚©‚çC•Ï”‚Ö‚ÌƒAƒNƒZƒX‚ð‹ÖŽ~‚·‚é
        if (stopped){// stopped == true‚È‚çƒ‹[ƒv‚ð”²‚¯‚é
            stopped = false;
            mutexSpd.unlock(); // ƒƒbƒN‰ðœ
            //            initSpd();
            break;
        }
        mutexSpd.unlock();

        //=================//
        //=== Main loop ===//
        //=================//

        //=== Start Timer ===//
        //===================//
        //qDebug() << time;
        if(flag.F_T == true){
            timer.startWatch();
        }
        mutexDq.lock();
        mutexDq.unlock();

        ////////////////////////////////



        ////////////////////////////////


        //=== Calculate Forward Dynamics ===//
        //=================================//
        f_dyn_rk(m,Gravity,d_time);

        //== Calc pos. & ori. of each joint w.r.t. link's fixed frame ==//
        //==============================================================//
        calc_SP(m); 

        //**********Joint Position**************//
        f_kin_j(m);

        //== Calc pos. & ori. of the selected end-effector ==//
        //===================================================//
        f_kin_e(m,e_num);

        //        //== Calc pos. & ori. of each joint w.r.t. link's fixed frame ==//
        //        //==============================================================//
        //        calc_SP(m);

        //        //== Calc pos. & ori. of the selected end-effector ==//
        //        //===================================================//
        //        f_kin_e(m,e_num);

        //        //**********Joint Position**************//
        //        f_kin_j(m);

        m.tau[1] = kd*(des_qd[0]-m.qd[1])+kp*(des_q[0]-m.q[1]);
        m.tau[2] = kd*(des_qd[1]-m.qd[2])+kp*(des_q[1]-m.q[2]);

        // ƒx[ƒX‚ÌŽp¨
        matrix_trans(3,3,m.A0,A0); //! A0=m.A0' ; dim(A0)=3x3
        dc2rpy(A0,Q0_rad); // SV.q0(3) == Q0_rad[2]

        // 20161009 <- ŠÔˆá‚¦‚Ä‚½
        //        rpy2dc(m.Q0,A0);
        //        matrix_trans(3, 3, A0, m.A0);

        //        for(int i = 0; i < 3; i++){
        //            Q0_rad[i] = m.Q0[i];
        //        }

        // Žèæ‚ÌŽp¨
        matrix_trans(3,3,m.ORI_e[1],ORI_e);
        dc2rpy(ORI_e, Qe_rad);
        for(int i = 0; i < 3; i++){
            Qe_deg[i] = pi2deg(Qe_rad[i]);
            Q0_deg[i] = pi2deg(Q0_rad[i]);
            POS_e[i] = m.POS_e[1][i];
        }

        // ŠÖßŠp“x
        q_deg[0] = pi2deg(m.q[1]);  //! For saving information
        q_deg[1] = pi2deg(m.q[2]);

        // ŠÖßŠp‘¬“x
        q_vel[0] = pi2deg(m.qd[1]);
        q_vel[1] = pi2deg(m.qd[2]);

        //=== Output simulation data to file ===//
        //======================================//
        if(flag.F_SD == true){// flag.F_SD‚ªtrue‚È‚çƒf[ƒ^•Û‘¶D•Û‘¶‚·‚éƒf[ƒ^‚Í"datasave.cpp"‚ÅŽw’è
            if(data_counter % 1 == 0){// ƒf[ƒ^‚ÌŠÔˆø‚«¨data_counter % 10 == 0 ‚È‚ç10‰ñ‚É1‰ñ•Û‘¶‚Æ‚¢‚¤‚±‚Æ
                dataSave.saveData(time);
            }
        }

        //=== Stop Timer ===//
        //==================//
        if(flag.F_T == true){
            timer.stopWatch();
            //  cout << timer.intervalT << "(msec)" << endl;
        }

        //=== Prepare for next step ===//
        //=============================//
        data_counter++;
        time += d_time;
        extime = time;
        //break;

        if(time > e_time){
            stopThreadSpd();
        }
    }
    //- Check thread end
    qDebug() << "Stop ThreadSpd";
    qDebug() << "Elapsed time = " << et.elapsed();
}
