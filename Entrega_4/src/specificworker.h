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
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
    void gotoTarget();
    bool atLine(RoboCompGenericBase::TBaseState bState);
    void bug(RoboCompGenericBase::TBaseState bState);

    bool obstacle();
    void targetAtSight();

    void turn();

	void setPick(const Pick &myPick);

public slots:
	void compute();

private:
        enum class State{IDLE, GOTO, BUG, TURN};
        State state = State::IDLE;
        struct {
            float P1X;
            float P1Z;
            float P2X;
            float P2Z;

            void setPoints(float _x1,float _z1,float _x2,float _z2){
                P1X=_x1;
                P1Z=_z1;
                P2X=_x2;
                P2Z=_z2;
            }
        }typedef Line2P;

    struct Target{
        void click(){
            QMutexLocker m1(&mutex);
           clicked=true;
        }

        void unClick(){
            QMutexLocker m1(&mutex);
           clicked=false;
        }

        void setX(float _x){
            QMutexLocker m1(&mutex);
            x=_x;
        }

        void setZ(float _z){
            QMutexLocker m1(&mutex);
            z=_z;
        }
        float getX(){
            return x;
        }

        float getZ(){
            return z;
        }
        bool isClicked(){

          return clicked;
        }
        void setVector(){
             QMutexLocker m1(&mutex);
           vec.setItem(0,x);
           vec.setItem(2,z);

        }
        QVec getVector(){
             QMutexLocker m1(&mutex);
            return vec;

        }

        float x=0.;
        float y=0.;
        float z=0.;
        QMutex mutex;
        QVec vec;
        bool clicked=false;
    };

	InnerModel *innerModel;
    Target tar;
    Line2P line2p;


};

#endif
