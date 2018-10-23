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
//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);



	return true;
}


void SpecificWorker::compute()

{

    float angle, distTarget, vadv, vrot;
        const float MaxAdv = 400;
        const float MaxRot = 0.5;
        const float e = 2.71828;
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState( bState);


    QVec x = QVec::vec2(tar.getX(), tar.getZ());
    Rot2D r( bState.alpha);
    QVec T = QVec::vec2(bState.x, bState.z);
    QVec y = r.invert() * (x - T);
    distTarget = y.norm2();
    qDebug()<<distTarget;
    angle = atan2(y.x(), y.z());
    qDebug()<<angle;
    qDebug()<<"La x"<<y.x();
    qDebug()<<"La z"<<y.z();

    if(distTarget<200){
       differentialrobot_proxy->setSpeedBase(0, 0);
       tar.unClick();

    }else{
        vadv = distTarget;
        vrot = angle;
        if(y.x()<200 && y.x()>-200)
        {
            vrot = 0;
            differentialrobot_proxy->setSpeedBase(400, vrot);
         }else{
            differentialrobot_proxy->setSpeedBase(0, vrot);
        }


    }








	//computeCODE
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}


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


   // std::cout << "Pos z del vector" << vector.z() << std::endl;




}


