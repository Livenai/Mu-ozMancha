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
/**
 * @brief SpecificWorker::laserFrontDist
 * @param ldata
 * @param threshold
 * @return
 */
bool SpecificWorker::laserFrontDist(TLaserData ldata, int threshold){
    for (int i=0;i<55;i++){
        if(ldata[20+i].dist<threshold){
            return true;
        }
    }
    return false;

}

/**
 * @brief SpecificWorker::isCorner
 * @param ldata
 * @param threshold
 * @return
 */
bool SpecificWorker::isCorner(TLaserData ldata, int threshold){
    int info=0;
    for (int i=0;i<99;i++){
        if(ldata[i].dist<threshold+200){
            info++;
        }
    }
    std::cout << info << endl;

    if(info>72){
       return true;

    }else{
       return false;
    }
}


/**
 * @brief SpecificWorker::speedFactor
 * @param ldata
 * @param MAX_DIST -> distancia la cual a partir de ella el factor siempre es 1
 * @return
 * method that returns a factor (from 0 to 1) as a function of the shortest distance detected
 * in the front. this factor is used to increase the speed as a function of the distance and
 * is worth 1 as long as the distance detected is greater than MAX_DIST
 */
float SpecificWorker::speedFactor(TLaserData ldata, float MAX_DIST)
{
    //seleccionamos del frente el que tenga menor distancia
    float minDist = 100000;
    for (int i = 0; i < 100; ++i) {
        if(isFront(i)){
            if(ldata[i].dist < minDist){
                minDist = ldata[i].dist;
            }
        }
    }
    //aplicamos la velocidad del robot en funcion a esta distancia
    float ret = (minDist/MAX_DIST);
    if(ret > 1)
        return 1;
    else
        return ret;
}


/**
 * @brief SpecificWorker::isFront
 * @param index -> posicion del vector
 * @return
 * Method that says if the position of the vector (index) given as a parameter
 * is considered part of the laser front. (the vector must have 100 elements)
 */
bool SpecificWorker::isFront(int index)
{
    if(index >= 35 && index <= 65){
        return true;
    } else {
        return false;
    }
}


/**
 * @brief SpecificWorker::normalTurn
 * @param ldata
 * @param rot -> rads/seg de giro
 * Method that performs the normal act of turning, understanding this by turning towards
 * the shortest side during a random time interval between 1.5 and 0.1 sec to rot rads / sec
 */
void SpecificWorker::normalTurn(TLaserData ldata, float rot)
{
    if(ldata.front().angle<0){
        differentialrobot_proxy->setSpeedBase(50, rot);
        usleep(rand()%(1000000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
    }else{
        differentialrobot_proxy->setSpeedBase(50, rot*-1);
        usleep(rand()%(1000000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
    }
}

/**
 * @brief SpecificWorker::specificTurn
 * @param ldata
 * @param rot -> rads/seg de giro
 * Method that performs the turn during a tick or execution. This method is used
 * to rotate as little as possible to overcome an obstacle in a critical situation
 * where the obstacle is too close
 */
void SpecificWorker::specificTurn(TLaserData ldata, float rot)
{
    if(ldata.front().angle<0){
        differentialrobot_proxy->setSpeedBase(50, rot);
        usleep(rand()%(1000000-100000 + 1) + 100000);
    }else{
        differentialrobot_proxy->setSpeedBase(50, rot*-1);
        usleep(rand()%(1000000-100000 + 1) + 100000);
    }
}




void SpecificWorker::compute( )
{
    //CONSTANTS
    const float threshold = 220; //millimeters
    const float MAX_SPEED = 1000; //max robot speed allowed
    const float MAX_DIST = 500; //from this distance in avance at MAX_SPEED
    const float MAX_ROT = 1.5;  //rads per second
    const float CRITIC_DIST = 200; //critical turn distace



    try {


        RoboCompLaser::TLaserData ldataSorted = laser_proxy->getLaserData();  //read laser data
        RoboCompLaser::TLaserData ldataRaw = laser_proxy->getLaserData();  //read laser data



        std::sort( ldataSorted.begin(), ldataSorted.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.


        float min_dist = ldataSorted.front().dist;
        if(min_dist < CRITIC_DIST){
            /*if critico. si un laser detecta una distancia menor que CRITIC_DIST
            * obliga a girar al robot para evitar choques inesperados */

            //gira la porcion minima necesaria para salvar el obstaculo
            specificTurn(ldataSorted,MAX_ROT);
        }else{
            if(isCorner(ldataRaw,threshold)){
                differentialrobot_proxy->setSpeedBase(0, MAX_ROT);
                usleep(2000000);
            }
            if(laserFrontDist(ldataRaw, threshold)){
                normalTurn(ldataSorted,MAX_ROT);
            }else{
                float speed = speedFactor(ldataRaw, MAX_DIST) * MAX_SPEED;
                differentialrobot_proxy->setSpeedBase(speed, 0);
            }
        } //else critico


    }catch(const Ice::Exception &ex){
        std::cout << ex << std::endl;
    }
}



