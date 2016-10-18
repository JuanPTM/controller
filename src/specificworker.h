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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>



class SpecificWorker : public GenericWorker
{
  
struct Target{
  mutable QMutex m;
  QVec pose = QVec(2);
  float angl;
  bool active = false;
  void setActive(bool newActive){
    QMutexLocker lm(&m);
    active = newActive;
  }
  void copy(float x, float z){
    QMutexLocker lm(&m);
    pose.setItem(0,x);
    pose.setItem(1,z);
  }
  QVec getPose(){
    QMutexLocker lm(&m);
    return pose;
  }
};

Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &mypick);

public slots:
	void compute(); 	

private:
	Target pick;
void dodge(int threshold,RoboCompLaser::TLaserData ldata);
	
};

#endif

