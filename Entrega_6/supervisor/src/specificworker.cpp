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
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    //       THE FOLLOWING IS JUST AN EXAMPLE
    //	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
    //	try
    //	{
    //		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
    //		std::string innermodel_path = par.value;
    //		innerModel = new InnerModel(innermodel_path);
    //	}
    //	catch(std::exception e) { qFatal("Error reading config params"); }


    timer.start(Period);


    return true;
}

void SpecificWorker::compute()
{
    QMutexLocker locker(mutex);
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



    ///inicializamos
    if(ESTABLECIDO){
        vectorDeMarcas[0].nombre = "marca1";
            vectorDeMarcas[0].x = -3143;
            vectorDeMarcas[0].z = 6000;

	vectorDeMarcas[1].nombre = "marca2";
            vectorDeMarcas[1].x = 2393;
            vectorDeMarcas[1].z = 4639;

	vectorDeMarcas[2].nombre = "marca3";
            vectorDeMarcas[2].x = 1970;
            vectorDeMarcas[2].z = -3000;

	vectorDeMarcas[3].nombre = "marca4";
            vectorDeMarcas[3].x = -5000;
            vectorDeMarcas[3].z = -2000;


        ESTABLECIDO = false;
        cout << "---------- PARAMS ESTABLECIDOS ----------" << endl;
    }

    cout << "---------- STATE-MACHINE ----------" << endl;
    switch(state){
        case State::SEARCH:
            //tiene que leer la camara mientras el robot gira, en busca de
            //la marca con el codigo currentTag.
            //una vez encontrada obtiene las coordenadas y llama al go() del chocachoca
            //para que avance. Mientras el supervisor pasa al estado WAIT
            gotopoint_proxy->go(vectorDeMarcas[currentTag].nombre, vectorDeMarcas[currentTag].x, vectorDeMarcas[currentTag].z, NULL);
            state = State::WAIT;

            break;
        case State::WAIT:
            //el supervisor espera y va preguntando (no se si con delay o de seguido)
            //si el robot ha llegado (cuando atTarget() del chocachoca devuelva true)
            //actualizamos currentTag (con el siguiente valor del XML con los diferentes tags)
            //y pasamos a SEARCH.
        if(gotopoint_proxy->atTarget()){
            nextTag();
            state = State::SEARCH;
        }

            break;
    }
}


void SpecificWorker::nextTag(){
    if(currentTag == 3){
        currentTag = 0;
    } else {
        currentTag++;
    }
}




