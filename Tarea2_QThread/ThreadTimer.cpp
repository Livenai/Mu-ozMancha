#include "ThreadTimer.h"

/**
 * @brief ThreadTimer::run
 * Metodo que ejecuta el hilo a iniciarse
 */
void ThreadTimer::run()
{
    //obtenemos el intervalo en segundos
    unsigned long intervaloSegundos = ((unsigned long) intervalo) / 1000;
    while(true){
        //esperamos el intervalo
        qDebug() << "tiempo segundos" << intervaloSegundos;

        sleep(intervaloSegundos);
        //y enviamos la señal
        emit timeout();
    }
}

ThreadTimer::ThreadTimer()
{
    //CONSTRUCTOR
    activo = false;
    intervalo = 1000;
}

ThreadTimer::~ThreadTimer()
{
    //DESTRUCTOR
}

/**
 * @brief ThreadTimer::start
 * metodo que inicia el contador con un intarvalo de tiempo en milisegundos
 */
void ThreadTimer::start(int ms)
{
 qDebug() << "start" << ms;
 intervalo = ms;
 activo = true;
 QThread::start();
}

/**
 * @brief ThreadTimer::isActive
 * @return true si activo; false si no
 * metodo que dice si el timer esta activo
 */
bool ThreadTimer::isActive()
{
    qDebug() << "isActive:" << activo;
    return activo;
}

/**
 * @brief ThreadTimer::stop
 * metodo que para el timer
 */
void ThreadTimer::stop()
{
    qDebug() << "stop";
    QThread::terminate();
    activo = false;
}



