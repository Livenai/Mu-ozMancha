/*
 *    Copyright (C)2018 by YOUR NAME HERE
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


    try
    {
        RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
        std::string innermodel_path = par.value;
        innerModel = new InnerModel(innermodel_path);
    }
    catch(std::exception e) { qFatal("Error reading config params"); }


	timer.start(Period);


	return true;
}

void SpecificWorker::compute()

{
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState( bState);
    innerModel->updateTransformValues("base", bState.x, 0, bState.z ,0, bState.alpha, 0 );


     switch( state )

    {

      case State::IDLE:

      if ( tar.isClicked() )
       {
       line2p.setPoints(bState.x,bState.z,tar.getX(), tar.getZ());
       state = State::GOTO;
        }
       break;

      case State::GOTO:

       gotoTarget();

       break;

      case State::BUG:

       bug();

       break;

      case State::TURN:
         turn();

         break;

     }



}


void SpecificWorker::gotoTarget()

 {

    if( obstacle() == true)   // If ther is an obstacle ahead, then transit to BUG

   {

      state = State::TURN;
      differentialrobot_proxy->setSpeedBase(0,0);


      return;

   }
    qDebug()<<"Calculando";

    QVec rt = innerModel->transform("base",QVec::vec3(tar.getX(), 0.,tar.getZ()),"world");
    float dist=rt.norm2();
    float angle = atan2(rt.x(), rt.z());
     if(angle<0.1 && angle > -0.1)
     {
          differentialrobot_proxy->setSpeedBase(500,0);
     }else{
         if(angle<0.1)
             differentialrobot_proxy->setSpeedBase(200,-1);
        else if(angle>-0.1)
             differentialrobot_proxy->setSpeedBase(200,1);

     }


   if(dist < 250)

  {
    qDebug()<< "Alcanzado";
    state = State::IDLE;
    differentialrobot_proxy->setSpeedBase(0,0);

    tar.unClick();

   return;

  }



 }
bool SpecificWorker::obstacle()
{
    qDebug()<<"Obstacle";

    TLaserData lsr = laser_proxy->getLaserData();
    std::sort( lsr.begin(), lsr.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;


    if(lsr.begin()->dist<250)
    {
         return true;
    }
    else
    return false;
}
void SpecificWorker::turn()
{
    qDebug()<<"Turning";
    TLaserData lsr = laser_proxy->getLaserData();
    bool closeDistance=false;
    for(int i=0;i<99;i++)
        if(lsr[i].dist<350){
          closeDistance=true;
        }

    if(!closeDistance){
       state = State::BUG;
        differentialrobot_proxy->setSpeedBase(0,0);

    }else{
        differentialrobot_proxy->setSpeedBase(0,-1);

    }


}
bool SpecificWorker::atLine(){
    qDebug()<<"At line";

    bool ret=false;
    float a= -line2p.P2Z -line2p.P1Z;
    float b= line2p.P2X -line2p.P1X;
    float c= -((a*line2p.P1X)+ (b*line2p.P1Z));
    c=fabs(c);
    if (c<1){
        ret=true;
    }

    return ret;
}
void SpecificWorker::bug()
{
    qDebug()<<"Bug";

    TLaserData lsr = laser_proxy->getLaserData();
    differentialrobot_proxy->setSpeedBase(250,0);

    if(lsr.begin()->dist<350){
        differentialrobot_proxy->setSpeedBase(250,-0.1);


    }
      if(lsr.begin()->dist>370){
          differentialrobot_proxy->setSpeedBase(250,0.1);

      }
      if(atLine()){
          state= State::GOTO;

      }



}

void SpecificWorker::targetAtSight()
{

}

void SpecificWorker::setPick(const Pick &myPick)
{
//subscribesToCODE

   if(!tar.isClicked()){
       tar.setX(myPick.x);
       tar.setZ(myPick.z);
       tar.click();
       tar.setVector();

   }

}
