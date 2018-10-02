#ifndef QTIMER
#define QTIMER


#include <QtGui> // para la salida por pantalla
#include <QObject>


class ThreadTimer : public QThread
{
    Q_OBJECT

signals:
    void timeout();

protected:
    void run();

public:
    ThreadTimer();
    virtual ~ThreadTimer();
    void start(int ms);
    bool isActive();
    void stop();

private:
    bool activo;
    int intervalo;

};



#endif // QTIMER
