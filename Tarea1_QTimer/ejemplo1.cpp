#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
    //CONSTRUCTOR
    contador = 0;
	setupUi(this);
    button->setText("START");
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
    connect(&T, SIGNAL(timeout()), this, SLOT(funcionActivacionTimer()));

}

ejemplo1::~ejemplo1()
{
    //DESTRUCTOR
}

void ejemplo1::doButton()
{
    //AL PULSAR EL BOTON
    //texto
    qDebug() << "el boton ha sido pulsado";
    if(button->text() == "STOP"){
        button->setText("START");
    } else {
        button->setText("STOP");
    }

    //accion
    if(T.isActive()){
         T.stop();
    } else {
         T.start(1000);
    }


}

void ejemplo1::funcionActivacionTimer()
{
    //LO QUE HARA ESTA CLASE AL PITAR EL TIMER
    contador++;
    this->lcdNumber->display(contador);
}




