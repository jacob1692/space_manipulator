//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : model( string, MODEL )
//            Read Model Parameters into Spatial Notation
//            from model file
//
// s.abiko [2007.5]
//
// modified by s.abiko [2008.10]
// Qi[0], JtoC[0], CtoJ[0] are initialized 
// modified by Jacob [2017.02]
// Take out the printing of elements of model.def except from LINKNUM and E_NUM 
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/spd/spd.h"

#define _LINESKIP ifs >> str;

void model( string filename, MODEL &m )
{
  int i, j;
  int check;
  string str;
  
  m.E_NUM = 0;
  
  ifstream ifs(filename.c_str());
  if( !ifs )
    {
      cerr << "Cannot open " << filename << endl;
      //exit(1);
    }
  
  _LINESKIP; //###Parameters_for_MODEL   
  ifs >> str >> m.LINKNUM;
  cout << endl;
  cout << "Loading and initializing "<<filename<< " in Space Dyn " << endl << endl;
  cout << " link_number = " << m.LINKNUM << endl;
  
  /* create arrays of MODEL class */
  m.constructor();
  
  _LINESKIP; //###Joint_Connectivity	
  if( m.LINKNUM != 1 )
    {
      ifs >> str;
    //  cout << " BB = [ ";
      for( i=1 ; i<m.LINKNUM; i++ ){
	ifs >> m.BB[i];
	//cout << m.BB[i] << " ";
      }
      ifs >> str;
    //  cout << " ] " << endl;
    }
 // cout << "\n";
  
  _LINESKIP; //###Joint_Type_[_0=rotational__1=prismatic]
  if( m.LINKNUM != 1 )
    {
      ifs >> str;
     // cout << " J_type = [ ";
      for( i=1 ; i<m.LINKNUM ; i++ ){
	ifs >> m.J_type[i];
	//cout << m.J_type[i] << " ";
	}
      ifs >> str;
     // cout << " ]" <<endl;
      //cout << "\n";
  
      _LINESKIP; //###EE
      ifs >> str;
     // cout << " EE = [ ";
      for( i=1 ; i<m.LINKNUM ; i++ )
	{
	  ifs >> m.EE[i];
    //      cout << m.EE[i] << " ";
	  if( m.EE[i] > m.E_NUM ) m.E_NUM++;
	}
   //   cout << " ]" <<endl;
      cout << " E_NUM=" << m.E_NUM <<endl;
   //   cout << "\n";
      
      ifs >> str;
      m.construct_ee( m.E_NUM );
    }
  
  
  // ========= added by s.abiko [2008.10] ==============
  	matrix_Z( 3, 1, m.CtoJ[0] );
	matrix_Z( 3, 1, m.JtoC[0] );
	matrix_Z( 3, 1, m.Qi[0] );
  // ===================================================
  
  _LINESKIP; //###Relative_Coordinate_System:Roll_Pitch_Yaw_Angle_of_Each_Bodies{x-y-z}
  if( m.LINKNUM != 1 )
    {
      for( i=1 ; i<m.LINKNUM ; i++ )
	{
	  ifs >> str >> m.Qi[i][0] >> m.Qi[i][1] >> m.Qi[i][2] >> str;
	  
	  m.Qi[i][0] = deg2pi(m.Qi[i][0]);
	  m.Qi[i][1] = deg2pi(m.Qi[i][1]);
	  m.Qi[i][2] = deg2pi(m.Qi[i][2]);
	  
	//  cout << " rpy"<< i << " = [ " <<  m.Qi[i][0] << " " << m.Qi[i][1] << " " << m.Qi[i][2] << " ]"<< endl;
	}
    //  cout << "\n";  
      _LINESKIP; //###Vector_Of_Link_Length
      for( i=1 ; i<m.LINKNUM ; i++ )
	{

	  ifs >> str >> m.CtoJ[i][0] >> m.CtoJ[i][1] >> m.CtoJ[i][2] >> str;
	  ifs >> str >> m.JtoC[i][0] >> m.JtoC[i][1] >> m.JtoC[i][2] >> str;
	//  cout << " CtoJ_"<< i << " = [ " <<  m.CtoJ[i][0] << " " << m.CtoJ[i][1] << " " << m.CtoJ[i][2] << " ]"<< endl;
     //     cout << "\n";
	 // cout << " JtoC_"<< i << " = [ " <<  m.JtoC[i][0] << " " << m.JtoC[i][1] << " " << m.JtoC[i][2] << " ]"<< endl;


	}
    }
  
  _LINESKIP; //###Vector_To_End-Effector
  if( m.E_NUM != 0 )
    for( i=0 ; i<m.E_NUM ; i++ ){
      ifs >> str >> m.CtoE[i][0] >> m.CtoE[i][1] >> m.CtoE[i][2] >> str;
    //  cout << " CtoE_"<< i+1 << " = [ " <<  m.CtoE[i][0] << " " << m.CtoE[i][1] << " " << m.CtoE[i][2] << " ]" << endl;
    }
  //cout << "\n";

  _LINESKIP; //###Relative_Coordinate_For_End-Effector
  if( m.E_NUM != 0 )
    for( i=0 ; i<m.E_NUM ; i++ ){
      ifs >> str >> m.Qe[i][0] >> m.Qe[i][1] >> m.Qe[i][2] >> str;
      
      m.Qe[i][0] = deg2pi(m.Qe[i][0]);
      m.Qe[i][1] = deg2pi(m.Qe[i][1]);
      m.Qe[i][2] = deg2pi(m.Qe[i][2]);
   //   cout << " rpyE_"<< i+1 << " = [ " <<  m.Qe[i][0] << " " << m.Qe[i][1] << " " << m.Qe[i][2] << " ]" << endl;
    }
 // cout << "\n";
  
  _LINESKIP; //###Mass_Parameter
  ifs >> str;
  for( i=0 ; i<m.LINKNUM ; i++ ){
    ifs >> m.link_M[i];
   // cout << " link_M" << i << " = " << m.link_M[i] << endl;
  }
 // cout << "\n";
  ifs >> str;
  
  _LINESKIP; //###Intertia_Matrix
  _LINESKIP; //###[_I11_I12_I13_]
  _LINESKIP; //###[_I12_I22_I23_]
  _LINESKIP; //###[_I13_I23_I33_]
  for( i=0 ; i<m.LINKNUM ; i++ )
    {
      ifs >> str;
      ifs >> str >> m.link_I[i][0*3+0];
      ifs >> str >> m.link_I[i][1*3+1];
      ifs >> str >> m.link_I[i][2*3+2];
      ifs >> str >> m.link_I[i][0*3+1]; m.link_I[i][1*3+0] = m.link_I[i][0*3+1];
      ifs >> str >> m.link_I[i][0*3+2]; m.link_I[i][2*3+0] = m.link_I[i][0*3+2];
      ifs >> str >> m.link_I[i][1*3+2]; m.link_I[i][2*3+1] = m.link_I[i][1*3+2];
    }
  /*
  for( i=0 ; i<m.LINKNUM ; i++ )
    {
      cout << "Inertia Matrix Link " << i << endl;
      for( j=0 ; j<3 ; j++ )
	cout << m.link_I[i][j*3+0]
	     << ' '
	     << m.link_I[i][j*3+1]
	     << ' '
	     << m.link_I[i][j*3+2] << endl;
      cout << endl;
    }
  */
  //###EOF
  ifs >> str >> check;
  if( check != 777 )
    {
      cerr << " #### error : model parameter ####" << endl;
      //exit(0);
    }
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  // convert all parameters to Spatial Notation
   calc_SPN( m );
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

  ifs.close();
}
// === EOF ===
