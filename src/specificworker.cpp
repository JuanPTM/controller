/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker ( MapPrx& mprx ) : GenericWorker ( mprx )
{
    state = State::INIT;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{
    innerModel= new InnerModel ( "/home/juanp/robocomp/files/innermodel/simpleworld.xml" );
    timer.start ( Period );

    return true;
}

void SpecificWorker::dodge ( int threshold, TLaserData ldata )
{
    /*
        if ( ldata[8].dist < threshold)
        {
            std::cout << ldata[8].dist << std::endl;
    	if (ldata[8].angle > 0)
    	{
    	  differentialrobot_proxy->setSpeedBase(5, -rot);
    	  usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
    	}else
    	{
    	  differentialrobot_proxy->setSpeedBase(5, rot);
    	  usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
    	}

        }
        else
        {
            differentialrobot_proxy->setSpeedBase(300, 0);
        }*/
    float rot = 0.6;

    if ( ldata[8].dist < threshold )
    {
        std::cout << ldata[8].dist << std::endl;
        if ( ldata[8].angle > 0 )
        {
            differentialrobot_proxy->setSpeedBase ( 5, -rot );
            usleep ( rand() % ( 1500000-100000 + 1 ) + 100000 ); //random wait between 1.5s and 0.1sec
        }
        else
        {
            differentialrobot_proxy->setSpeedBase ( 5, rot );
            usleep ( rand() % ( 1500000-100000 + 1 ) + 100000 ); //random wait between 1.5s and 0.1sec
        }

    }
}

void SpecificWorker::compute()
{


    const float threshold = 420; //millimeters
    float rot = 0.6;  //rads per second

    try
    {

        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
        RoboCompDifferentialRobot::TBaseState bState;
        differentialrobot_proxy->getBaseState ( bState );
        innerModel->updateTransformValues ( "base",bState.x,0,bState.z,0,bState.alpha,0 );

        switch ( state )
        {
        case State::INIT:
	      if ( pick.active )
		  state=State::GOTO;
	      break;
	  case State::GOTO:
	      movement ( ldata );
	      break;
	  case State::BUGINIT:
	      buginit(ldata);
	      break;
	  case State::BUG:
	      bugMovement (ldata);
	      break;
	  case State::END:
	      break;
	  }
    }
    catch ( const Ice::Exception &ex )
    {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::movement ( const TLaserData &tLaser )
{
    QVec tr = innerModel->transform ( "base",pick.getPose(),"world" );
    
    float angle = atan2 ( tr.x(),tr.z() );
    float distance = tr.norm2();

    if ( distance <= 100 )
    {
        pick.setActive ( false );
	qDebug() << "FINISH: GOTO TO INIT";
        state= State::INIT;
        differentialrobot_proxy->stopBase();
    }

    if ( obstacle ( tLaser ) )
    {
        state=State::BUGINIT;
	qDebug() << "GOTO to BUGINIT";
        return;
    }

    if ( abs ( angle ) > 0.05 )
      distance = 0;
    if( distance > 300) distance = 300;
    
    try
    {
      differentialrobot_proxy->setSpeedBase(distance, angle);
    }
    catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
 }


void SpecificWorker::buginit ( const TLaserData& ldata )
{
  if( obstacle(ldata) == false)
  {
    state = State::BUG;
    qDebug() << "change to BUG";
    return;
  }
  
  try
  {
      differentialrobot_proxy->setSpeedBase(0, 0.3);
  }
  catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
}


void SpecificWorker::bugMovement ( const TLaserData &ldata )
{
    const float alpha = log ( 0.1 ) /log ( 0.3 ); //amortigua /corte
    
//     if ( targetAtSight ( ldata ) )  //CAMBIAR POR CROSSLINE
//     {
//         state = State::GOTO;
// 	qDebug() << "from BUG to GOTO";
//         return;
//     }
    
    if ( obstacle (ldata) ){
        state = State::BUGINIT;
 	qDebug() << "from BUG to BUGINIT";
        return;
    }

    float dist = ldata[90].dist;
    qDebug() << dist;

    //vr = -( 1.0/800 ) * dist + 0.5; 
    float vrot =  -((1./(1. + exp(-(dist - 450.))))-1./2.);
    qDebug()<< vrot<<" rotacion";
    float vadv = 350 * exp ( - ( fabs ( vrot ) * alpha ) ); // QLin2D
    qDebug()<< vadv<<" avance";
    differentialrobot_proxy->setSpeedBase ( vadv ,vrot );

}

bool SpecificWorker::targetAtSight ( TLaserData ldata )
{
    QPolygon poly;
    for ( auto l: ldata )
    {
        QVec r = innerModel->laserTo ( "world","laser",l.dist,l.angle );
        QPoint p ( r.x(),r.z() );
        poly<<p;

    }
    return poly.containsPoint ( QPoint ( pick.getPose().x(),pick.getPose().z() ), Qt::OddEvenFill );
}


//AUX

void SpecificWorker::stopRobot()
{
  try
  {
      differentialrobot_proxy->stopBase();
  }
  catch ( const Ice::Exception &ex )
  {
      std::cout << ex << std::endl;
  }
}

bool SpecificWorker::obstacle ( TLaserData tLaser )
{
    std::sort ( tLaser.begin() +35, tLaser.end()-35, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b )
    {
        return     a.dist < b.dist;
    } ) ; //sort laser data from small to large distances using a lambda function.

    return ( tLaser[35].dist < 400 );
}

//////////////////////////
//// SERVANTS FOR ICE INTERFACE
///////////////////////////

void SpecificWorker::setPick ( const Pick &mypick )
{
    qDebug() <<mypick.x<<mypick.z;
    pick.copy ( mypick.x,mypick.z );
    pick.setActive ( true );
    state = State::INIT;
}




//FUSCA



        /*
            if(pick.active)
        {


          float baseAngle=bState.alpha;
          float targetX = pick.getPose().getItem(0);
          float targetZ = pick.getPose().getItem(1);
          float baseX = bState.x;
          float baseZ = bState.z;

          float M[2][2];
          M[0][0] = cos(baseAngle);
          M[1][0]= sin(baseAngle);
          M[0][1]= -sin(baseAngle);
          M[1][1]= cos(baseAngle);
          float Tr[2];
          Tr[0] = M[0][0] * (targetX - baseX) + M[0][1]* (targetZ - baseZ);
          Tr[1] = M[1][0] * (targetX - baseX) + M[1][1]* (targetZ - baseZ);

          float angle = atan2(Tr[0],Tr[1]);
          double x = (targetX-baseX);
          double z = (targetZ-baseZ);
          double distance = sqrt((x*x)+(z*z));
          //TODO calcular avance y hacerlo en un if (frenandose al avanzar)

          qDebug()<<angle<<"Angulo angle";

          if (abs(angle) <= 0.005)
          {
            if(ldata[8].dist < threshold)
            {
              dodge(threshold,ldata);
            }
            differentialrobot_proxy->setSpeedBase(distance*0.5,0);
          }else
            differentialrobot_proxy->setSpeedBase(0,rot*angle);

          if (distance<= threshold/2)
          {
              pick.setActive(false);
              differentialrobot_proxy->stopBase();
          }
        } */

