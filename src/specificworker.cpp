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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);
	

	return true;
}

void SpecificWorker::setPick(const Pick &mypick){
  qDebug()<<"Recibido mypick";
  qDebug()<<mypick.x<<mypick.z;
  pick.copy(mypick.x,mypick.z);
  pick.setActive(true);
  enfocado=false;
}

void SpecificWorker::compute()
{
    
    const float threshold = 420; //millimeters
    float rot = 0.6;  //rads per second
 
// 400 x 400
    try
    {
      
      
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
        std::sort( ldata.begin()+8, ldata.end()-8, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.

        RoboCompDifferentialRobot::TBaseState bState;
	
        if(pick.active)
	{
	  differentialrobot_proxy->getBaseState( bState);
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
	  if(!enfocado)
	  {

	    qDebug()<<angle<<"Angulo angle";
	  
	    if (abs(angle) <= 0.05)
	    {
	      differentialrobot_proxy->stopBase();
	      enfocado = true;
	    }else
	      if(angle >0)
		differentialrobot_proxy->setSpeedBase(0,rot*angle);
	      else
		differentialrobot_proxy->setSpeedBase(0,rot*angle);
	  }
	  
	  if (enfocado)
	  {
	    double x = (targetX-baseX);
	    double z = (targetZ-baseZ); //TODO pow, y eliminar qDebug.
	    double distance = sqrt((x*x)+(z*z));
	    qDebug()<<distance<<targetX<<baseX<<x<<targetZ<<baseZ<<z;
	    differentialrobot_proxy->setSpeedBase(distance*0.5,0);
	    if (distance<= threshold/2)
	    {
	      pick.setActive(false);
	      enfocado = false;
 	      differentialrobot_proxy->stopBase();
	      float giro =((angle+baseAngle)%(6.28));
	      qDebug()<<giro;
	    }
	  }
	}
    /*if( ldata[8].dist < threshold)
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
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}







