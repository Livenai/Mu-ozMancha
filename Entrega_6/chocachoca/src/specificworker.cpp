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
Line2P line2p;

/**
* \brief Default constructor
*/


SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    std::cout << std::boolalpha;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    try
    {
        RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
        innerModel = std::make_shared<InnerModel>(par.value);
    }
    catch(std::exception e) { qFatal("Error reading config params"); }

    qDebug() << __FILE__ ;

    // Scene
    scene.setSceneRect(-5000, -5000, 10000, 10000);
    view.setScene(&scene);
    view.scale(1, -1);
    view.setParent(scrollArea);
    //view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
    view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );

    grid.initialize( TDim{ 50, -8000, 7000, -6000, 8000}, TCell{0, true, false, nullptr, 0.} );

    for(auto &[key, value] : grid)
    {
        auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));
        tile->setPos(key.x,key.z);
        value.rect = tile;
    }

    robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
    noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
    noserobot->setBrush(Qt::magenta);

    target = QVec::vec3(0,0,0);

    //qDebug() << __FILE__ << __FUNCTION__ << "CPP " << __cplusplus;

    connect(buttonSave, SIGNAL(clicked()), this, SLOT(saveToFile()));
    connect(buttonRead, SIGNAL(clicked()), this, SLOT(readFromFile()));

    timer.start();

    return true;
}

void SpecificWorker::compute()
{
    static RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);

        //draw robot
        robot->setPos(bState.x, bState.z);
        robot->setRotation(-180.*bState.alpha/M_PI);



        if(targetReady)
        {

            if(planReady)
            {

                if(path.empty())
                {
                    qDebug() << "Arrived to target";
                    targetReady = false;
                    enDestino = true;
                    camera=false;
                    differentialrobot_proxy->setSpeedBase(0,0);
                }
                else{

                    currentPoint = path.front();
                    QVec rt = innerModel->transform("base",currentPoint,"world");

                    float dist=rt.norm2();
                    float angle = atan2(rt.x(), rt.z());
                    if(angle<0.1 && angle > -0.1)
                    {
                        differentialrobot_proxy->setSpeedBase(300,0);
                    }else{
                        if(angle<0.1)
                            differentialrobot_proxy->setSpeedBase(0,-1);
                        else if(angle>-0.1)
                            differentialrobot_proxy->setSpeedBase(0,1);

                    }

                    if(dist <50)
                    {
                        currentPoint = path.front();
                        path.pop_front();
                    }



                }



            }
            else
            {
                qDebug() << bState.x << bState.z << target.x() << target.z() ;
                path = grid.getOptimalPath(QVec::vec3(bState.x,0,bState.z), target);
                for(auto &p: path)
                    greenPath.push_back(scene.addEllipse(p.x(),p.z(), 100, 100, QPen(Qt::green), QBrush(Qt::green)));
                planReady = true;
            }
        }
    }
    catch(const Ice::Exception &e)
    {	std::cout  << e << std::endl; }

    //Resize world widget if necessary, and render the world
    if (view.size() != scrollArea->size())
        view.setFixedSize(scrollArea->width(), scrollArea->height());
    draw();

}
void SpecificWorker::gotoTarget(RoboCompGenericBase::TBaseState bState)
{



    /*  if( obstacle() == true)   // If ther is an obstacle ahead, then transit to BUG

    {

        state = State::TURN;
        differentialrobot_proxy->setSpeedBase(0,0);


        return;

    }*/

    QVec rt = innerModel->transform("base",currentPoint,"world");
    float dist=rt.norm2();
    float angle = atan2(rt.x(), rt.z());
    if(angle<1 && angle > -1)
    {
        differentialrobot_proxy->setSpeedBase(300,0);
    }else{
        if(angle<1)
            differentialrobot_proxy->setSpeedBase(0,-1);
        else if(angle>-1)
            differentialrobot_proxy->setSpeedBase(0,1);

    }



    if(dist < 320){
        state = State::IDLE;
        differentialrobot_proxy->setSpeedBase(0,0);
        //tar.unClick();
        return;
    }
}

bool SpecificWorker::atLine(RoboCompGenericBase::TBaseState bState){
    qDebug()<<"At line";
    qDebug()<<"------ COOR: " << line2p.P1X <<" "<< line2p.P1Z <<" "<< line2p.P2X <<" "<< line2p.P2Z;
    bool ret=false;


    float a= line2p.P1Z -line2p.P2Z;
    float b= line2p.P2X -line2p.P1X;
    float c= (line2p.P1X * (a*-1)) - (line2p.P1Z * b);


    double dist = fabs((a*bState.x + b*bState.z + c))/sqrt((a*a) + (b*b));
    //double dist = ((a*bState.x) + (b*bState.z) + c);
    //dist = fabs(dist);
    qDebug()<<"------ dist: " << dist;
    if (dist<150){
        qDebug()<<"LINE CROSS--------------------------------> " << dist;
        ret=true;
    }
    //falla al calcular la linea,pues el origen esta bien pero el destino es el primero pinchado


    return true;
}




void SpecificWorker::saveToFile()
{
    grid.saveToFile(fileName);
}

void SpecificWorker::readFromFile()
{
    std::ifstream myfile;
    myfile.open(fileName, std::ifstream::in);

    if(!myfile.fail())
    {
        //grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{true, false, nullptr} );
        for( auto &[k,v] : grid)
            delete v.rect;
        grid.clear();
        Grid<TCell>::Key key; TCell value;
        myfile >> key >> value;
        int k=0;
        while(!myfile.eof())
        {
            auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));;
            tile->setPos(key.x,key.z);
            value.rect = tile;
            value.id = k++;
            value.cost = 1;
            grid.insert<TCell>(key,value);
            myfile >> key >> value;
        }
        myfile.close();
        robot->setZValue(1);
        std::cout << grid.size() << " elements read to grid " << fileName << std::endl;
    }
    else
        throw std::runtime_error("Cannot open file");
}

void SpecificWorker::checkTransform(const RoboCompGenericBase::TBaseState &bState)
{
    auto r = innerModel->transform("base", target, "world");		// using InnerModel

    Rot2D rot(bState.alpha);																		// create a 2D clockwise rotation matrix
    QVec t = QVec::vec2(bState.x, bState.z);									  // create a 2D vector for robot translation
    QVec t2 = QVec::vec2(target.x(), target.z());								// create a 2D vector from the 3D target
    QVec q = rot.transpose() * ( t2 - t);												// multiply R_t * (y - T)
    qDebug() << target << r << q;
}

void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
/*
    auto *n = innerModel->getNode<InnerModelLaser>("laser");
    for(auto l: ldata)
    {
        auto r = n->laserTo("world", l.dist, l.angle);	// r is in world reference system
        // we set the cell corresponding to r as occupied
        auto [valid, cell] = grid.getCell(r.x(), r.z());
                if(valid)
                cell.free = false;
    }*/
    }
                void SpecificWorker::updateVisitedCells(int x, int z){
            static unsigned int cont = 0;
            auto [valid, cell] = grid.getCell(x, z);
                    if(valid)
            {
                auto &occupied = cell.visited;
                if(occupied)
                {
                    occupied = false;
                    cont++;
                }
                float percentOccupacy = 100. * cont / grid.size();
            }
        }

        void SpecificWorker::draw()
        {
            for(auto &[key, value] : grid)
            {
                // 		if(value.visited == false)
                // 			value.rect->setBrush(Qt::lightGray);
                if(value.free == false)
                    value.rect->setBrush(Qt::darkRed);
            }
            view.show();
        }



        void SpecificWorker::setPick(const Pick &myPick)
        {
            target[0] = myPick.x;
            target[2] = myPick.z;
            target[1] = 0;
            qDebug() << __FILE__ << __FUNCTION__ << myPick.x << myPick.z ;
            targetReady = true;
            planReady = false;
            for(auto gp: greenPath)
                delete gp;
            greenPath.clear();

        }


        ///////////// METODOS DE LA INTEERFAZ /////////////

        void SpecificWorker::go(const string &nodo, const float x, const float y, const float alpha)
        {
            //guardar nombre y coordenadas del destino y
            //prepara las cosas para ejecutar el camino
            target[0] = x;
            target[2] = y;
            target[1] = 0;
            NombreDestino = nodo;
            cout << "Funcion Go(" << NombreDestino << ", " << x << ", " << y << ");\n";
            targetReady = true;
            planReady = false;
            enDestino = false;
            for(auto gp: greenPath)
                delete gp;
            greenPath.clear();
        }

        void SpecificWorker::turn(const float speed)
        {
            //mientras marcaLista este a false debe girar 
	    while(marcaLista == false){
		differentialrobot_proxy->setSpeedBase(0,1);
	    }
        }

        bool SpecificWorker::atTarget()
        {
            //devuelve si se ha llegado al destino
            return enDestino;
        }

        void SpecificWorker::stop()
        {
            //para el robot y restaura el estado para esperar una nueva orden
            differentialrobot_proxy->setSpeedBase(0,0);
            targetReady = false;
            planReady = false;
            enDestino = false;
        }

        /////////////////////////////    AprilTags    /////////////////////////////

	void SpecificWorker::newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)
	{
	//subscribesToCODE

	}

	void SpecificWorker::newAprilTag(const tagsList &tags)
	{
	//subscribesToCODE
	    cout << "El tags es ::: "<< tags[0].id << " x "<< tags[0].tx << " y "<< tags[0].ry << " z "<< tags[0].rz << " cameraid "<< tags[0].cameraId<< endl;
	    marcaLista = true;
	/*
	   if(camera==false){
	    target[0] = tags[0].tx;
	    target[2] = tags[2].tz;
	    target[1] = 0;
	   // qDebug() << __FILE__ << __FUNCTION__ << myPick.x << myPick.z ;
	    targetReady = true;
	    camera=true;
	    planReady = false;
	    for(auto gp: greenPath)
		delete gp;
	    greenPath.clear();
	    }
	*/
	}


	



